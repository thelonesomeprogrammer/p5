/*
 * Admittance-Controlled Robotic Foot System
 * 
 * This system implements cascaded admittance and PID control for a robotic foot
 * with force sensors. The foot measures forces and adjusts its position using
 * a virtual mass-spring-damper model (admittance control) followed by PID
 * position tracking.
 */

#define DT_WINDOW_SIZE 5
// ============================================================================
// Data Structures
// ============================================================================
/**
 * PID Controller Structure
 * Implements a discrete-time PID controller with anti-windup
 */
typedef struct  {
  float kp;          // Proportional gain
  float ki;          // Integral gain
  float kd;          // Derivative gain
  float integral;    // Accumulated integral term
  float last_errors[DT_WINDOW_SIZE];  // Previous error for derivative calculation
} pid_t;

/**
 * Admittance Controller Structure
 * Implements a virtual mass-spring-damper (MSD) system that generates
 * desired positions based on measured forces
 */
  typedef struct  {
  float k;         // Spring stiffness [N/m]
  float c;         // Damping coefficient [N·s/m]
  float inv_m;     // Inverted virtual mass [1/kg]
  float spring_zero; // Rest position of the virtual spring [rad]
  float virt_pos;  // Previous position for velocity estimation [rad]
  float virt_vel;  // Previous velocity for acceleration estimation [rad/s]
} admittance_t;

/**
 * Second-order Butterworth Filter Structure
 * Used for low-pass filtering force sensor readings to remove noise
 */
typedef struct {
  float b0;
  float b1;
  float b2;
  float a1;
  float a2;
  float x1; // Previous input (n-1)
  float x2; // Previous input (n-2)
  float y1; // Previous output (n-1)
  float y2; // Previous output (n-2)
} butter_t;

/**
 * Exponential Moving Average (Low-Pass) Filter
 * Simple first-order filter for smoothing
 */
typedef struct {
  float last;  // Previous filtered value
  float alpha; // Smoothing factor (0-1)
}exp_t;


// ============================================================================
// Global constants
// ============================================================================
const int dt_window_size = DT_WINDOW_SIZE;
const float upper_lim = 3.25;
const float lower_lim = 1.75;

// Sensor configuration
const int apins[] = {A1, A2, A3, A4, A5, A6, A7, A8, A9, A10};  // 10 analog pins for 5 sensor pairs

// ============================================================================
// Global Variables
// ============================================================================

// Time tracking
long dt_times[DT_WINDOW_SIZE];
int dt_index;

// Assist factor
float helpfactor = 1.0;


// Controller instances
admittance_t m_msd;  // Admittance controller (outer loop)
pid_t pid;           // PID controller (inner loop)

// System state variables
float des;                      // Desired position
int speed;                      // Motor PWM speed command [-255, 255]
int p;                          // Counter for periodic logging

butter_t butter_force[5];
butter_t butter_force_2;

// Force sensor data
int basefoot[] = {0, 0, 0, 0, 0};  // Baseline readings for 5 sensor pairs (tared values)
float newfoot[]  = {0.0, 0.0, 0.0, 0.0, 0.0};  // Current force readings for 5 sensor pairs [N]

// Moving average filter for force sensors
long moveavg_sum[5] = {0};  // Running sum for efficient average calculation
int movavg_vals[5] = {0};


// ============================================================================
// Setup Function
// ============================================================================

/**
 * System Initialization
 * Configures serial communication, pin modes for sensors and motors,
 * initializes controller parameters (PID and Admittance), and tares
 * the force sensors to establish a zero baseline.
 */
void setup() {
  // Initialize serial communication for debugging
  Serial.begin(115200);

  // Configure analog input pins
  pinMode(A0, INPUT);   // Position sensor (potentiometer or encoder)
  
  // Force sensor pairs (differential measurements)
  pinMode(A1, INPUT);   // Pair 0 - Left
  pinMode(A2, INPUT);   // Pair 0 - Right
  pinMode(A3, INPUT);   // Pair 1 - Left
  pinMode(A4, INPUT);   // Pair 1 - Right
  pinMode(A5, INPUT);   // Pair 2 - Left
  pinMode(A6, INPUT);   // Pair 2 - Right
  pinMode(A7, INPUT);   // Pair 3 - Left
  pinMode(A8, INPUT);   // Pair 3 - Right
  pinMode(A9, INPUT);   // Pair 4 - Left
  pinMode(A10, INPUT);  // Pair 4 - Right
  
  // Configure motor control pins (H-bridge PWM)
  pinMode(8, OUTPUT);   // Motor direction A
  pinMode(9, OUTPUT);   // Motor direction B


  // Initialize PID controller gains
  // Tuned for position tracking with reasonable response
  pid.kp = 140;                    // Proportional gain
  pid.ki = 0.8 * pid.kp;       // Integral gain
  pid.kd = 0.2 * pid.kp;         // Derivative gain
  pid.integral = 0.0;              // Clear integral accumulator

  for (int i = 0; i < 10; i++){
    pid.last_errors[i] = 0.;            // Initialize error history
  }


  // Initialize admittance controller parameters
  // These define the virtual mechanical impedance of the system
  m_msd.k = 0.04;                               // Low stiffness for compliant behavior
  m_msd.c = 0.5;                                      // Moderate damping
  m_msd.inv_m = 1.0/0.005;                                     // Virtual inertia (inverse mass) 
  m_msd.virt_pos = analogRead(A0) * 0.007 + 0.1655;// Initialize with current position
  m_msd.virt_vel = 0.;                               // Start at rest
  m_msd.spring_zero = 2.5;                     // Neutral spring position

  des = m_msd.virt_pos;

  // Tare force sensors - record baseline readings with no applied force
  // Fill moving average buffers with initial readings
  for (int i = 0; i < 10; i++) {
    for (int j = 0; j < 5; j++) {
      fill_weat(j);
    }
  }
  // Store baseline values
  for (int i = 0; i < 5; i++) {
    basefoot[i] = fill_weat(i);

    butter_force[i].b0 =  0.00208460;
    butter_force[i].b1 =  0.00416920;
    butter_force[i].b2 =  0.00208460;
    butter_force[i].a1 = -1.86675938;
    butter_force[i].a2 =  0.87509779;
  }

  butter_force_2.b0 =  0.00208460;
  butter_force_2.b1 =  0.00416920;
  butter_force_2.b2 =  0.00208460;
  butter_force_2.a1 = -1.86675938;
  butter_force_2.a2 =  0.87509779;

  // Warm up filter 
  for (int j = 0; j < 100; j++){
    for (int i = 0; i < 5; i++) {
      read_weat(i);
    }
    butter_step(butter_force_2, m_msd.virt_pos);
  }

  dt_index = 0;
  long now = millis();
  for(int i=0; i<DT_WINDOW_SIZE; i++) {
    dt_times[i] = now;
  }

  // logging
  p = 0;
}

// ============================================================================
// Main Control Loop
// ============================================================================

/**
 * Main Control Loop
 * Executes the primary control cycle:
 * 1. Reads position and calculates timing.
 * 2. Reads force sensors.
 * 3. Runs Admittance Control (Force -> Desired Position).
 * 4. Runs PID Control (Position Error -> Motor Command).
 * 5. Handles safety checks and motor actuation.
 */
void loop() {
  p++;
  // Read current position from potentiometer/encoder
  int read = analogRead(A0);

  // Convert ADC reading to physical position (radians)
  // Formula: position = read * 0.005 + 0.0565
  float rad = read * 0.007 + 0.1655;

  rad = butter_step(&butter_force_2, rad);


  long t = millis();
  // Calculate time step over the full window (10 samples)
  float dt10 = (t - dt_times[dt_index])/1000.0;
  
  // Calculate inverse of window time for derivative calculation (dE/dt10)
  float odt10 = 1.0/dt10; 

  // Calculate time step since last loop iteration
  // (dt_index + 9) % 10 gives the index of the most recent previous sample
  float dt = (t - dt_times[(dt_index + dt_window_size - 1) % dt_window_size])/1000.0;
  
  dt_times[dt_index] = t;

  // Read all force sensors (5 pairs, 10 sensors total)
  for (int i = 0; i < 5; i++) {
    newfoot[i] = read_weat(i);
  }

    if (p % 51 == 0){
    // Log data periodically for debugging/tuning
    log(rad);
  }

  // Safety check: stop motor if position is out of valid range
  // Prevents damage if sensor disconnects or reaches mechanical limits
  if (rad > upper_lim || rad < lower_lim) {
    speed = 0;
    analogWrite(8, 0);
    analogWrite(9, 0);
    return;
  }

  if (p % dt_window_size == 0) {

    // Control cascade:
    // 1. Calculate net torque/force from sensor readings
    float f = force();
    
    
    // 2. Admittance control: force -> desired position
    //    Virtual MSD system generates compliant response to external forces
    admittance(f, dt10);
  }

  float err = des - rad; 

  // 3. PID control: position error -> motor command
  //    Tracks the desired position
  speed = pidstep(err, dt, odt10);
    
  // Limit motor speed to safe range
  speed = constrain(speed, -60, 60);
  if (speed > -17 && speed < 17){speed = 0;}
  
  
  // Apply motor command via H-bridge
  // PWM on pins 8 and 9 controls direction and speed
  if (speed == 0) {
    // Brake: both outputs low
    analogWrite(8, 0);
    analogWrite(9, 0);
  } else if (speed < 0) {
    // Reverse: drive pin 9, pin 8 low
    analogWrite(9, -speed);
    analogWrite(8, 0);
  } else {
    // Forward: drive pin 8, pin 9 low
    analogWrite(9, 0);
    analogWrite(8, speed);
  }

  dt_index = (dt_index + 1) % dt_window_size;
  if (p == 100){p=0;}
}

// ============================================================================
// Filters
// ============================================================================

/**
 * Apply one step of the Second-Order Butterworth Filter
 * 
 * @param butter Pointer to the filter state structure
 * @param input  New raw input value
 * @return       Filtered output value
 */
float butter_step(butter_t* butter, float input) {
  float output = butter->b0*input + butter->b1*butter->x1 + butter->b2*butter->x2 - butter->a1*butter->y1 - butter->a2*butter->y2;
        
  butter->x2 = butter->x1; 
  butter->x1 = input;
  butter->y2 = butter->y1; 
  butter->y1 = output;
        
  return output;
}

/**
 * Apply one step of the Exponential Moving Average Filter
 * 
 * @param fil    Pointer to the filter state structure
 * @param input  New raw input value
 * @return       Filtered output value
 */
float exp_step(exp_t* fil, float input) {
  float output = fil->alpha * (input - fil->last) + fil->last;
  fil->last = output;
  return output;
}




// ============================================================================
// PID Controller
// ============================================================================

/**
 * Execute one PID control step
 * 
 * @param m_pid  Pointer to PID controller structure
 * @param error  Position error (desired - actual)
 * @param dt     Time step in seconds
 * 
 * Implements the discrete PID equation:
 * u(t) = Kp*e(t) + Ki*∫e(τ)dτ + Kd*de(t)/dt
 *
 * @return       Control output (motor command)
 */
float pidstep(float error, float dt, float odt) {
  // Integrate error over time
  pid.integral += error * dt;
  
  // Anti-windup: clamp integral to prevent excessive buildup
  pid.integral = constrain(pid.integral, -0.2, 0.2);

  // Calculate derivative (rate of change of error)
  // Note: Uses error from 10 samples ago (dt_window_size) to smooth out the derivative
  // term and reduce noise sensitivity. odt is 1/dt10.
  float derivative = (error - pid.last_errors[dt_index]) * odt;

  // Store error for next derivative calculation
  pid.last_errors[dt_index] = error;

  // Compute PID output: proportional + integral + derivative terms
  return pid.kp * error + pid.ki * pid.integral + pid.kd * derivative;
}

// ============================================================================
// Force Sensor Reading
// ============================================================================

/**
 * Read and process force from a Wheatstone bridge sensor pair
 * 
 * Each "pair" consists of two Wheatstone bridge sensors in a differential
 * configuration. This provides better noise rejection and sensitivity.
 * 
 * @param pair  Sensor pair index (0-4)
 * @return      Filtered force reading in Newtons
 */
float read_weat(int pair) {
  // Read both sensors in the differential pair
  int read = analogRead(apins[pair * 2]);        // Left sensor
  int read2 = analogRead(apins[pair * 2 + 1]);   // Right sensor

  int pN = read - read2;
  int s = -12 * pN + 5582;
  // Subtract baseline (tare) to get relative force
  int current = s - basefoot[pair];
  float avg = current;  // Return filtered average
  avg = butter_step(&butter_force[pair], avg);
  
  // Convert grams to Newtons (g/1000)
  avg *= 0.0098066500286389;
  return avg;
}

/**
 * Calibrate/Tare a Wheatstone bridge sensor pair
 * 
 * Reads the sensor pair and updates a running average to establish
 * the baseline zero-force value.
 * 
 * @param pair  Sensor pair index (0-4)
 * @return      Current average baseline value
 */
int fill_weat(int pair) {
    // Read both sensors in the differential pair
  int read = analogRead(apins[pair * 2]);        // Left sensor
  int read2 = analogRead(apins[pair * 2 + 1]);   // Right sensor

  int pN = read - read2;  
  
  moveavg_sum[pair] += pN;
  movavg_vals[pair] += 1;

  
  int avg = moveavg_sum[pair] / movavg_vals[pair];
  return -12 * avg + 5582;
}

// ============================================================================
// Admittance Controller
// ============================================================================

/**
 * Execute Admittance Control Step
 * 
 * Calculates the desired position of the foot based on the measured force
 * using a virtual Mass-Spring-Damper model.
 * F = m*a + c*v + k*x  =>  a = (F - c*v - k*x) / m
 * 
 * @param force  Net force/torque acting on the foot
 * @param dt10   Time step for integration
 */
void admittance(float force, float dt10) {

    // 1. Calculate the "Spring" and "External" forces first (excluding damping)
    float spring_force = m_msd.k * (m_msd.virt_pos - m_msd.spring_zero);
    float acc_no_damping = (force - spring_force) * m_msd.inv_m;

    // 2. Calculate the velocity denominator for Implicit Damping
    // Denom = 1 + (c * dt / m)
    float denom = 1.0 + (m_msd.c * dt10 * m_msd.inv_m);

    // 3. Update Velocity (Implicitly)
    // v_new = (v_old + dt * a_no_damping) / denom
    m_msd.virt_vel = (m_msd.virt_vel + acc_no_damping * dt10) / denom;
    
    // Safety Clamp for Velocity
    m_msd.virt_vel = constrain(m_msd.virt_vel, -2.0, 2.0);

    // 4. Update Position (Standard Semi-Implicit)
    m_msd.virt_pos += m_msd.virt_vel * dt10;
    
    // Safety Clamp for Position
    m_msd.virt_pos = constrain(m_msd.virt_pos, lower_lim, upper_lim);
    
    // Set global desired position
    des = m_msd.virt_pos;
}

// ============================================================================
// Force Calculation
// ============================================================================

/**
 * Calculate net torque/force from all sensors
 * 
 * Computes a weighted moment about the foot's pivot point.
 * Front sensors (0-2) and back sensors (3-4) create opposing torques.
 * 
 * @return  Net torque or normalized force
 */
float force()  {
  // Average force from front sensors (3 sensors)
  float front_avg = (newfoot[0] + newfoot[1] + newfoot[2]) * 0.33 * helpfactor;
  
  // Average force from back sensors (2 sensors)
  float back_avg = (newfoot[3] + newfoot[4]) * 0.5 * helpfactor;

  // Moment arm lengths [m]
  float flen = 0.11;  // Front sensor distance from pivot
  float blen = 0.07;   // Back sensor distance from pivot

  // Calculate the net force immediately
  // Positive pushes back, Negative pushes front
  float net_force = (front_avg * flen) - (back_avg * blen);

  return net_force*2;
}


// ============================================================================
// help factor calculation
// ============================================================================
void help() {
  Serial.println("starting assist level calibration");
  m_msd.k = 3;
  float forces[500] = {0.};

  for (int i = 0; i > 500; i++){
    loop();
    forces[i] = force();
  }

  for ( int i = 2; i < 498; i++){
    forces[i] = (forces[i-2] + forces[i-1] + forces[i] + forces[i+1] + forces[i+2]) / 5.0;
  }
  float maxf = 0.;
  for ( int i = 0; i > 500; i++){
    if (abs(forces[i]) > maxf){
      maxf = abs(forces[i]);
    }
  }
  Serial.print("max force detected: ");
  Serial.println(maxf);
  helpfactor = 10.0 / maxf;
  Serial.print("help factor set to: ");
  Serial.println(helpfactor);
  m_msd.k = 0.005;
}


// ============================================================================
// Debug Logging
// ============================================================================

/**
 * Periodically log system state for debugging and tuning
 * 
 * Prints every ~500 loop iterations to avoid overwhelming serial buffer
 * 
 * @param output  Admittance controller output
 * @param rad    Current position [rad]
 */
void log(float rad) {
  // Print sensor readings
  Serial.print("millis: ");
  Serial.print(millis());

    
  Serial.print(", top1:");      // Front sensors
  Serial.print(newfoot[0]);
  Serial.print(",top2:");
  Serial.print(newfoot[1]);
  Serial.print(",top3:");
  Serial.print(newfoot[2]);
  Serial.print(",but1:");    // Back sensors ("butt" / rear)
  Serial.print(newfoot[3]);
  Serial.print(",but2:");
  Serial.print(newfoot[4]);

  Serial.print(", vpos: ");
  Serial.print(m_msd.virt_pos);
  Serial.print(", vvel: ");
  Serial.print(m_msd.virt_vel);

  // Print desired position
  Serial.print(", des: ");
  Serial.print(des);
    
  // Print position
  Serial.print(", rad: ");
  Serial.print(rad);
    
  // Print final motor command
  Serial.print(", con: ");
  Serial.println(speed);
}
