/*
 * Admittance-Controlled Robotic Foot System
 * 
 * This system implements cascaded admittance and PID control for a robotic foot
 * with force sensors. The foot measures forces and adjusts its position using
 * a virtual mass-spring-damper model (admittance control) followed by PID
 * position tracking.
 */

#define DT_WINDOW_SIZE 10
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
  float m;         // Virtual mass [kg]
  float last_pos[DT_WINDOW_SIZE];  // Previous position for velocity estimation [rad]
  float last_vel[DT_WINDOW_SIZE];  // Previous velocity for acceleration estimation [rad/s]
  float last_acc[DT_WINDOW_SIZE];
} admittance_t;

typedef struct {
  float b0;
  float b1;
  float b2;
  float a1;
  float a2;
  float x1;
  float x2;
  float y1;
  float y2;
} butter_t;

typedef struct {
  float last;
  float alpha;
}exp_t;


// ============================================================================
// Global constents
// ============================================================================
const int dt_window_size = DT_WINDOW_SIZE;

// Sensor configuration
const int apins[] = {A1, A2, A3, A4, A5, A6, A7, A8, A9, A10};  // 10 analog pins for 5 sensor pairs

// ============================================================================
// Global Variables
// ============================================================================

// dt 
long dt_times[DT_WINDOW_SIZE];
int dt_index;


// Controller instances
admittance_t m_msd;  // Admittance controller (outer loop)
pid_t pid;           // PID controller (inner loop)

// System state variables
float des;                      // Desired position
int speed;                      // Motor PWM speed command [-255, 255]
int p;                          // Counter for periodic logging


butter_t butter_force[5];
butter_t butter_vel;

// Force sensor data
int basefoot[] = {0, 0, 0, 0, 0};  // Baseline readings for 5 sensor pairs (tared values)
float newfoot[]  = {0.0, 0.0, 0.0, 0.0, 0.0};  // Current force readings for 5 sensor pairs [N]

// Moving average filter for force sensors
long moveavg_sum[5] = {0};  // Running sum for efficient average calculation
int movavg_vals[5] = {0};


// moving average for msd
float msdavg_buf[10] = {0};
int msdavg_idx = 0;
float madavg_sum = 0;

// ============================================================================
// Setup Function
// ============================================================================

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
  pid.kp = 130;                    // Proportional gain
  pid.ki = 0.535 * pid.kp;       // Integral gain (13.5% of Kp)
  pid.kd = 0.135 * pid.kp;         // Derivative gain (130% of Kp for damping)
  pid.integral = 0.0;              // Clear integral accumulator


  // Initialize admittance controller parameters
  // These define the virtual mechanical impedance of the system
  m_msd.k = 0.3; // 1/k                               // Low stiffness for compliant behavior
  m_msd.c = 0.005;                                      // Moderate damping
  m_msd.m = 0.00;                                     // Virtual inertia 

  for (int i = 0; i < 10; i++){
    m_msd.last_pos[i] = analogRead(A0) * 0.005 + 0.0565;// Initialize with current position
    m_msd.last_vel[i] = 0.;                               // Start at rest
    m_msd.last_acc[i] = 0.;

    pid.last_errors[i] = 0.;            // Initialize error history
    delay(2);
  }

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

    butter_force[i].b0 =  0.0000878299618;
    butter_force[i].b1 =  0.0001756599240;
    butter_force[i].b2 =  0.0000878299618;
    butter_force[i].a1 = -1.97331757;
    butter_force[i].a2 =  0.97366889;
  }

  butter_vel.a1 = -1.9111882;
  butter_vel.a2 =  0.9149677;
  butter_vel.b0 =  0.00094488;
  butter_vel.b1 =  0.00188975;
  butter_vel.b2 =  0.00094488;

  

  // logging
  p = 0;
}

// ============================================================================
// Main Control Loop
// ============================================================================

void loop() {
  // Read current position from potentiometer/encoder
  int read = analogRead(A0);
  long t = millis();
  float dt = (t - dt_times[dt_index])/1000.0;
  float odt = 1/dt;
  dt_times[dt_index] = t;

  Serial.print(t);
  Serial.print(",");
  Serial.print(dt,4);
  Serial.print(",");
  
  
  // Safety check: stop motor if position is out of valid range
  // Prevents damage if sensor disconnects or reaches mechanical limits
  //if (read > 400 || read < 70) {
  if (1==0){
    Serial.println("bounds");
    speed = 0;
  } else {
    // Convert ADC reading to physical position (radians)
    // Formula: position = read * 0.005 + 0.0565
    float rad = read * 0.005 + 0.0565;
    

    // Read all force sensors (5 pairs, 10 sensors total)
    for (int i = 0; i < 5; i++) {
      newfoot[i] = read_weat(i);
    }

    // Control cascade:
    // 1. Calculate net torque/force from sensor readings
    float f = force();
    
    // 2. Admittance control: force -> desired position
    //    Virtual MSD system generates compliant response to external forces
    float output = admittance(f, rad, odt);
    madavg_sum -= msdavg_buf[msdavg_idx];      // Remove oldest sample
    msdavg_buf[msdavg_idx] = output;           // Add new sample
    madavg_sum += output;                      // Update running sum
    msdavg_idx = (msdavg_idx + 1) % 10;         // Advance circular index
    des = madavg_sum * 0.1;                    // Smoothed desired position
    
    Serial.print(output,6);
    Serial.println(des,6);

    // 3. PID control: position error -> motor command
    //    Tracks the desired position
    speed = pidstep(des, dt, odt);
    
    // Log data periodically for debugging/tuning
    log(rad);
    
    // Limit motor speed to safe range
    speed = constrain(speed, -60, 60);
    if (speed > -15 && speed < 15){speed = 0;}
  }
  
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
}

// ============================================================================
// Filters
// ============================================================================

float butter_step(butter_t* butter, float input) {
  float output = butter->b0*input + butter->b1*butter->x1 + butter->b2*butter->x2 - butter->a1*butter->y1 - butter->a2*butter->y2;
        
  butter->x2 = butter->x1; 
  butter->x1 = input;
  butter->y2 = butter->y1; 
  butter->y1 = output;
        
  return output;
}

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
  pid.integral = constrain(pid.integral, -0.05, 0.05);

  // Calculate derivative (rate of change of error)
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
 * Read and process force from a sensor pair
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

  int pN = (read - read2) * -15;
  // Subtract baseline (tare) to get relative force
  int current = pN - basefoot[pair];
  current = constrain(current, -50000, 50000);
  

  
  
  float avg = current;  // Return filtered average
  Serial.print(current);
  Serial.print(",");
  avg = butter_step(&butter_force[pair], avg);
  Serial.print(avg,4);
  Serial.print(",");
  avg *= 0.00488758;
  return avg;
}

int fill_weat(int pair) {
    // Read both sensors in the differential pair
  int read = analogRead(apins[pair * 2]);        // Left sensor
  int read2 = analogRead(apins[pair * 2 + 1]);   // Right sensor

  int pN = (read - read2) * -15;  
  
  moveavg_sum[pair] += pN;
  movavg_vals[pair] += 1;

  
  return moveavg_sum[pair] / movavg_vals[pair];
}

// ============================================================================
// Admittance Controller
// ============================================================================

/**
 * Admittance control: computes desired position from measured force
 * 
 * Implements a virtual mass-spring-damper system:
 * m*ẍ + c*ẋ + k*x = F_external
 * 
 * Solving for desired position:
 * x_desired = (F - c*ẋ - m*ẍ) * 1/k
 * 
 * @param force  External force measured by sensors [N]
 * @param pos    Current position [rad]
 * @param dt     Time step [s]
 *
 * @return       Desired position [rad]
 */
float admittance(float force, float pos, float odt) {

  //Serial.print(pos,4);
  //Serial.print(",");

  // Estimate velocity from position derivative
  float vel = (pos - m_msd.last_pos[dt_index]) * odt;
  //Serial.print(vel,4);
  //Serial.print(",");
  vel = butter_step(&butter_vel, vel);
  //Serial.print(vel,4);
  //Serial.print(",");
  if (vel <= 0.01 && vel >= -0.01) {vel = 0;}
  
  // Estimate acceleration from velocity derivative
  float acc = (vel - m_msd.last_vel[dt_index]) * odt;
  //Serial.println(acc,4);
  if (acc <= 0.01 && acc >= -0.01) {acc = 0;}

  // Update state history for next iteration
  m_msd.last_pos[dt_index] = pos;
  m_msd.last_vel[dt_index] = vel;
  m_msd.last_acc[dt_index] = acc;

  // Calculate desired position using admittance equation
  // This creates a compliant response to external forces
  return (-m_msd.c * vel - m_msd.m * acc + force) * m_msd.k;
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
  float front = (newfoot[0] + newfoot[1] + newfoot[2]) * 0.333333;
  
  // Average force from back sensors (2 sensors)
  float back = (newfoot[3] + newfoot[4]) * 0.5;
  
  // Moment arm lengths [cm]
  float flen = 11.0;  // Front sensor distance from pivot
  float blen = 7.0;   // Back sensor distance from pivot
  
  // Calculate net moment and normalize by total lever arm
  // Positive = front-loaded, Negative = back-loaded
  return (front * flen - back * blen) * 0.0555555;
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
  // Only log every 500 iterations to reduce serial bandwidth
  if (p > 50) {
    p = 0;
    
    // Print sensor readings
    //Serial.print("millis: ");
    //Serial.print(millis());

    
    //Serial.print("top1:");      // Front sensors
    //Serial.print(newfoot[0]);
    //Serial.print(",top2:");
    //Serial.print(newfoot[1]);
    //Serial.print(",top3:");
    //Serial.print(newfoot[2]);
    //Serial.print(",but1:");    // Back sensors ("butt" / rear)
    //Serial.print(newfoot[3]);
    //Serial.print(",but2:");
    //Serial.println(newfoot[4]);

    // print desiered
    //Serial.print(", des: ");
    //Serial.print(des);
    
    // Print position
    //Serial.print(", rad: ");
    //Serial.print(rad);
    
    // Print final motor command
    //Serial.print(", con: ");
    //Serial.println(speed);
  }
  p++;  // Increment logging counter
}
