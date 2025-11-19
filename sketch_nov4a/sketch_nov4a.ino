/*
 * Admittance-Controlled Robotic Foot System
 * 
 * This system implements cascaded admittance and PID control for a robotic foot
 * with force sensors. The foot measures forces and adjusts its position using
 * a virtual mass-spring-damper model (admittance control) followed by PID
 * position tracking.
 */

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
  float last_error;  // Previous error for derivative calculation
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
  float last_pos;  // Previous position for velocity estimation [rad]
  float last_vel;  // Previous velocity for acceleration estimation [rad/s]
} admittance_t;


// ============================================================================
// Global constents
// ============================================================================
const float dt = 0.1;
const float odt = 1.0/dt;



// ============================================================================
// Global Variables
// ============================================================================

// Controller instances
admittance_t m_msd;  // Admittance controller (outer loop)
pid_t pid;           // PID controller (inner loop)

// System state variables
float des;                      // Desired position
int speed;                      // Motor PWM speed command [-255, 255]
int p;                          // Counter for periodic logging

// Sensor configuration
int apins[] = {A1, A2, A3, A4, A5, A6, A7, A8, A9, A10};  // 10 analog pins for 5 sensor pairs

// Force sensor data
int basefoot[] = {0, 0, 0, 0, 0};  // Baseline readings for 5 sensor pairs (tared values)
float newfoot[]  = {0.0, 0.0, 0.0, 0.0, 0.0};  // Current force readings for 5 sensor pairs [N]

// Moving average filter for force sensors
int moveavg[5][50] = {0};  // Circular buffer for each sensor pair (50 samples)
int moveavg_idx[5] = {0};    // Current index in circular buffer for each pair
int moveavg_sum[5] = {0};  // Running sum for efficient average calculation

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
  pid.ki = 0.135 * pid.kp;       // Integral gain (13.5% of Kp)
  pid.kd = 1.3 * pid.kp;         // Derivative gain (130% of Kp for damping)
  pid.integral = 0.0;              // Clear integral accumulator
  pid.last_error = 0.0;            // Initialize error history

  // Initialize admittance controller parameters
  // These define the virtual mechanical impedance of the system
  m_msd.k = 0.5; // 1/k                               // Low stiffness for compliant behavior
  m_msd.c = 10;                                      // Moderate damping
  m_msd.m = 15;                                     // Virtual inertia
  m_msd.last_pos = analogRead(A0) * 0.005 + 0.0565;  // Initialize with current position
  m_msd.last_vel = 0;                               // Start at rest

  // Tare force sensors - record baseline readings with no applied force
  // Fill moving average buffers with initial readings
  for (int i = 0; i < 50; i++){
    for (int j = 0; j < 5; j++) {
      fill_weat(j);
    }
  }
  // Store baseline values
  for (int i = 0; i < 5; i++) {
    basefoot[i] = fill_weat(i);
  }

  

  // logging
  p = 0;
}

// ============================================================================
// Main Control Loop
// ============================================================================

void loop() {
  // Read current position from potentiometer/encoder
  int read = analogRead(A0);
  
  // Safety check: stop motor if position is out of valid range
  // Prevents damage if sensor disconnects or reaches mechanical limits
  if (read > 400 || read < 70) {
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
    float output = admittance(f, rad);
    madavg_sum -= msdavg_buf[msdavg_idx];      // Remove oldest sample
    msdavg_buf[msdavg_idx] = output;           // Add new sample
    madavg_sum += output;                      // Update running sum
    msdavg_idx = (msdavg_idx + 1) % 10;         // Advance circular index
    des = madavg_sum * 0.1;                    // Smoothed desired position
    
    // 3. PID control: position error -> motor command
    //    Tracks the desired position
    speed = pidstep(des);
    
    // Log data periodically for debugging/tuning
    log(rad);
    
    // Limit motor speed to safe range
    speed = constrain(speed, -30, 30);
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
float pidstep(float error) {
  // Integrate error over time
  pid.integral += error * dt;
  
  // Anti-windup: clamp integral to prevent excessive buildup
  pid.integral = constrain(pid.integral, -0.5, 0.5);

  // Calculate derivative (rate of change of error)
  float derivative = (error - pid.last_error) * odt;

  // Store error for next derivative calculation
  pid.last_error = error;

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
  if (abs(current) < 45)  {current = 0;}
  
  // Apply moving average filter (31-sample window) for noise reduction
  // This uses a circular buffer for efficient O(1) updates
  moveavg_sum[pair] -= moveavg[pair][moveavg_idx[pair]];  // Remove oldest sample
  moveavg[pair][moveavg_idx[pair]] = current;             // Add new sample
  moveavg_sum[pair] += current;                           // Update running sum
  moveavg_idx[pair] = (moveavg_idx[pair] + 1) % 50;       // Advance circular index
  
  float avg = moveavg_sum[pair] * 0.02 * 0.00488758;  // Return filtered average
  return avg;
}

int fill_weat(int pair) {
    // Read both sensors in the differential pair
  int read = analogRead(apins[pair * 2]);        // Left sensor
  int read2 = analogRead(apins[pair * 2 + 1]);   // Right sensor

  int pN = (read - read2) * -15;  
  
  // Apply moving average filter (31-sample window) for noise reduction
  // This uses a circular buffer for efficient O(1) updates
  moveavg_sum[pair] -= moveavg[pair][moveavg_idx[pair]];  // Remove oldest sample
  moveavg[pair][moveavg_idx[pair]] = pN;             // Add new sample
  moveavg_sum[pair] += pN;                           // Update running sum
  moveavg_idx[pair] = (moveavg_idx[pair] + 1) % 50;       // Advance circular index
  
  return moveavg_sum[pair] * 0.02;
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
float admittance(float force, float pos) {
  // Estimate velocity from position derivative
  float vel = (pos - m_msd.last_pos);
  
  // Estimate acceleration from velocity derivative
  float acc = (vel - m_msd.last_vel);

  // Update state history for next iteration
  m_msd.last_pos = pos;
  m_msd.last_vel = vel;

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
  if (p > 100) {
    p = 0;
    
    // Print sensor readings
    Serial.print("millis: ");
    Serial.print(millis());

    
    Serial.print("; top: ");      // Front sensors
    Serial.print(newfoot[0]);
    Serial.print(", ");
    Serial.print(newfoot[1]);
    Serial.print(", ");
    Serial.print(newfoot[2]);
    Serial.print("; but: ");    // Back sensors ("butt" / rear)
    Serial.print(newfoot[3]);
    Serial.print(", ");
    Serial.print(newfoot[4]);

    // print desiered
    Serial.print("; des: ");
    Serial.print(des);
    
    // Print position
    Serial.print("; rad: ");
    Serial.print(rad);
    
    // Print final motor command
    Serial.print("; con: ");
    Serial.println(speed);
  }
  p++;  // Increment logging counter
}
