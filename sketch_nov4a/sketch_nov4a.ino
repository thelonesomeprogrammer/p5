/*
 * Admittance-Controlled Robotic Foot System
 * 
 * This system implements cascaded admittance and PID control for a robotic foot
 * with force sensors. The foot measures forces and adjusts its position using
 * a virtual mass-spring-damper model (admittance control) followed by PID
 * position tracking.
 */

#define DT_WINDOW_SIZE 5
#define MEDIAN_WINDOW_SIZE 5
// ============================================================================
// Data Structures
// ============================================================================
/**
 * PID Controller Structure
 * Implements a discrete-time PID controller with anti-windup
 */
typedef struct {
  float kp;                           // Proportional gain
  float ki;                           // Integral gain
  float kd;                           // Derivative gain
  float integral;                     // Accumulated integral term
  float last_errors[DT_WINDOW_SIZE];  // Previous error for derivative calculation
} pid_t;

/**
 * Admittance Controller Structure
 * Implements a virtual mass-spring-damper (MSD) system that generates
 * desired positions based on measured forces
 */
typedef struct {
  float k;            // Spring stiffness [N/m]
  float c;            // Damping coefficient [N·s/m]
  float inv_m;        // Inverted virtual mass [1/kg]
  float denom;        // Damping denominator for implicit integration [unitless]
  float spring_zero;  // Rest position of the virtual spring [rad]
  float virt_pos;     // Previous position for velocity estimation [rad]
  float virt_vel;     // Previous velocity for acceleration estimation [rad/s]
} admittance_t;

/**
 * Second-order Filter Structure
 * Used for low-pass filtering force sensor readings to remove noise
 */
typedef struct {
  float b0;
  float b1;
  float b2;
  float a0;
  float a1;
  float a2;
  float x1;  // Previous input (n-1)
  float x2;  // Previous input (n-2)
  float y1;  // Previous output (n-1)
  float y2;  // Previous output (n-2)
} secfilt_t;

/**
 * Log Data Structure
 * Holds a snapshot of system state variables for periodic logging
 */
typedef struct {
  unsigned long time; // Timestamp [ms]
  float avg_loop;     // Average loop duration [us]
  float top1;         // Force sensor reading 1 [N]
  float top2;         // Force sensor reading 2 [N]
  float top3;         // Force sensor reading 3 [N]
  float bot1;         // Force sensor reading 4 [N]
  float bot2;         // Force sensor reading 5 [N]
  float virt_pos;     // Virtual position [rad]
  float virt_vel;     // Virtual velocity [rad/s]
  float rad;          // Actual position [rad]
  float f;            // Net force [Nm]
  int control_out;    // Motor control output (PWM)
  int index;          // Serialization state index
} log_t;

/**
 * Exponential Moving Average (Low-Pass) Filter
 * Simple first-order filter for smoothing
 */
typedef struct {
  float last;   // Previous filtered value
  float alpha;  // Smoothing factor (0-1)
} exp_t;


/**
 * Median Filter Structure
 * Implements a median filter using a sorted array approach for efficient outlier removal
 */
typedef struct {
  int buffer[MEDIAN_WINDOW_SIZE];  // Circular buffer for raw values
  int sorted[MEDIAN_WINDOW_SIZE];  // Buffer sorted for median extraction
  int idx;                         // Current write index
  int window_size; // Size of the window
} median_t;


// ============================================================================
// Global constants
// ============================================================================
const int dt_window_size = DT_WINDOW_SIZE;
const float upper_lim = 3.25;
const float lower_lim = 1.75;
const long LOOP_INTERVAL_US = 2500;
const float dt = LOOP_INTERVAL_US / 1e6;  // Loop time step in seconds
const float dt10 = dt * dt_window_size;   // Time step for admittance and PID derivative
const float odt10 = 1.0 / dt10;           // Inverse of dt10 for efficiency

// Sensor configuration
const int apins[] = { A1, A2, A3, A4, A5, A6, A7, A8, A9, A10 };  // 10 analog pins for 5 sensor pairs

// ============================================================================
// Global Variables
// ============================================================================

// Time tracking
unsigned long previous_micros = 0;
int dt_index = 0;

long loop_counter = 0;

// Assist factor
float help_factor = 2.0;

// logging
log_t last_log;

// Controller instances
admittance_t msd;  // Admittance controller (outer loop)
pid_t pid;           // PID controller (inner loop)

// System state variables
float des;  // Desired position
int speed;  // Motor PWM speed command [-255, 255]

secfilt_t filt_force;
secfilt_t filt_pos;


median_t medians[5];

// Force sensor data
int base_foot[] = { 0, 0, 0, 0, 0 };                 // Baseline readings for 5 sensor pairs (tared values)
float new_foot[] = { 0.0, 0.0, 0.0, 0.0, 0.0 };      // Current force readings for 5 sensor pairs [N]

// Moving average filter for force sensors
long movavg_sum[5] = { 0 };  // Running sum for efficient average calculation
int movavg_vals[5] = { 0 };


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
  pinMode(A0, INPUT);  // Position sensor (potentiometer or encoder)

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
  pinMode(8, OUTPUT);  // Motor direction A
  pinMode(9, OUTPUT);  // Motor direction B


  // Initialize PID controller gains
  // Tuned for position tracking with reasonable response
  pid.kp = 160;           // Proportional gain
  pid.ki = 2.5 * pid.kp;  // Integral gain
  pid.kd = 0.2 * pid.kp;  // Derivative gain
  pid.integral = 0.0;     // Clear integral accumulator

  for (int i = 0; i < dt_window_size; i++) {
    pid.last_errors[i] = 0.;  // Initialize error history
  }


  // Initialize admittance controller parameters
  // These define the virtual mechanical impedance of the system
  msd.k = 0.04;                                    // Low stiffness for compliant behavior
  msd.c = 0.5;                                     // Moderate damping
  msd.inv_m = 1.0 / 0.005;                         // Virtual inertia (inverse mass)
  msd.virt_pos = analogRead(A0) * 0.007 + 0.1655;  // Initialize with current position
  msd.virt_vel = 0.;                               // Start at rest
  msd.spring_zero = 2.5;                           // Neutral spring position
  msd.denom = 1.0 + (msd.c * dt10 * msd.inv_m);    // pre-calculate denominator 

  des = msd.virt_pos;

  // Tare force sensors - record baseline readings with no applied force
  // Fill moving average buffers with initial readings
  for (int i = 0; i < 10; i++) {
    for (int j = 0; j < 5; j++) {
      calibrate_fsr(j);
    }
  }
  // Store baseline values
  for (int i = 0; i < 5; i++) {
    base_foot[i] = calibrate_fsr(i);
    medians[i].idx = 0;
    medians[i].window_size = MEDIAN_WINDOW_SIZE;
    // Initialize median filter buffers
    for (int j = 0; j < MEDIAN_WINDOW_SIZE; j++) {
      medians[i].buffer[j] = base_foot[i];
      medians[i].sorted[j] = base_foot[i];
    }
  }

  filt_pos.b0 = 0.00761985;
  filt_pos.b1 = 0.0152397;
  filt_pos.b2 = 0.00761985;
  filt_pos.a0 = 1.0;
  filt_pos.a1 = -1.69028083;
  filt_pos.a2 = 0.72076023;

  filt_force.b0 = 0.00144327;
  filt_force.b1 = 0.00288653;
  filt_force.b2 = 0.00144327;
  filt_force.a0 = 1.0;
  filt_force.a1 = -1.86697805;
  filt_force.a2 = 0.87275111;

  // Warm up filter
  for (int j = 0; j < 100; j++) {
    for (int i = 0; i < 5; i++) {
      read_fsr(i);
    }
    float f = force();
    secfilt_step(&filt_force, f);
    secfilt_step(&filt_pos, msd.virt_pos);
  }
  
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
  unsigned long current_micros = micros();

  if (current_micros - previous_micros < LOOP_INTERVAL_US) {
    log();
    return;
  }

  previous_micros += LOOP_INTERVAL_US;

  // Read current position from potentiometer/encoder
  int read = analogRead(A0);

  // Convert ADC reading to physical position (radians)
  // Formula: position = read * 0.005 + 0.0565
  float rad = read * 0.007 + 0.1655;

  rad = secfilt_step(&filt_pos, rad);

  // Read all force sensors (5 pairs, 10 sensors total)
  for (int i = 0; i < 5; i++) {
    new_foot[i] = read_fsr(i);
  }

  // Calculate net torque/force from sensor readings 
  float f = force();
  f = secfilt_step(&filt_force, f);


  if (loop_counter % 50 == 0) {
    // capture logs every ~125ms
    Serial.println();
    capture_log(rad, f);
  }

  // Safety check: stop motor if position is out of valid range
  // Prevents damage if sensor disconnects or reaches mechanical limits
  if (rad > upper_lim || rad < lower_lim) {
    speed = 0;
    analogWrite(8, 0);
    analogWrite(9, 0);
    return;
  }

  if (loop_counter % dt_window_size == 0) {

    //    Admittance control: force -> desired position
    //    Virtual MSD system generates compliant response to external forces
    admittance_step(f, dt10);
  }

  float err = des - rad;

  // 3. PID control: position error -> motor command
  //    Tracks the desired position
  speed = pid_step(err, dt, odt10);

  // Limit motor speed to safe range
  speed = constrain(speed, -60, 60);
  
  // Deadband to prevent motor buzzing/stalling at low speeds
  if (speed >  5 && speed < 20) { speed = 20; }
  if (speed <  -5 && speed > -20) { speed = -20; }


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

  loop_counter++;

  // Check for loop overruns
  if (micros() - current_micros > LOOP_INTERVAL_US) {
    Serial.println("Loop overrun!");
  }
}

// ============================================================================
// Filters
// ============================================================================


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

/**
 * Apply one step of the Second-order Filter (Biquad)
 *
 * Implements a Direct Form I structure for an IIR filter:
 * y[n] = (b0*x[n] + b1*x[n-1] + b2*x[n-2] - a1*y[n-1] - a2*y[n-2]) / a0
 *
 * @param fil    Pointer to the filter state structure
 * @param input  New raw input value
 * @return       Filtered output value
 */
float secfilt_step(secfilt_t* fil, float input) {
  float output = (fil->b0 * input + fil->b1 * fil->x1 + fil->b2 * fil->x2 - fil->a1 * fil->y1 - fil->a2 * fil->y2) / fil->a0;

  fil->x2 = fil->x1;
  fil->x1 = input;
  fil->y2 = fil->y1;
  fil->y1 = output;

  return output;
}

/**
 * Apply one step of the Median Filter
 *
 * Maintains a sliding window of values and returns the median.
 * Uses an optimized insertion sort approach on the rolling buffer.
 *
 * @param fil    Pointer to the filter state structure
 * @param input  New raw input value
 * @return       Median value from the current window
 */
float median_step(median_t* fil, int input) {
  // 1. Identify the old value we are about to overwrite
  int old = fil->buffer[fil->idx];

  // 2. Overwrite it in the circular buffer
  fil->buffer[fil->idx] = input;
  fil->idx = (fil->idx + 1) % fil->window_size;

  // 3. UPDATE SORTED:
  // Instead of re-sorting the whole array, we find the old value
  // in the sorted array, replace it with the new value, and re-sort just that element.
  int i;
  // Find old value in sorted array
  for (i = 0; i < fil->window_size; i++) {
    if (fil->sorted[i] == old) break;
  }

  // Replace with new value
  fil->sorted[i] = input;

  // Bubble this single new value to its correct position
  // Move right if larger
  while (i < fil->window_size - 1 && fil->sorted[i] > fil->sorted[i + 1]) {
    int temp = fil->sorted[i];
    fil->sorted[i] = fil->sorted[i + 1];
    fil->sorted[i + 1] = temp;
    i++;
  }
  // Move left if smaller
  while (i > 0 && fil->sorted[i] < fil->sorted[i - 1]) {
    int temp = fil->sorted[i];
    fil->sorted[i] = fil->sorted[i - 1];
    fil->sorted[i - 1] = temp;
    i--;
  }

  // 4. Return the middle element
  return fil->sorted[fil->window_size / 2];
}


// ============================================================================
// Controller
// ============================================================================

/**
 * Execute one PID control step
 * 
 * @param error  Position error (desired - actual)
 * @param dt     Time step in seconds
 * @param odt    Inverse time step for derivative calculation (1/dt)
 * 
 * Implements the discrete PID equation:
 * u(t) = Kp*e(t) + Ki*∫e(τ)dτ + Kd*de(t)/dt
 *
 * @return       Control output (motor command)
 */
float pid_step(float error, float dt, float odt) {
  // Integrate error over time
  pid.integral += error * dt;

  // Anti-windup: clamp integral to prevent excessive buildup
  pid.integral = constrain(pid.integral, -0.025, 0.025);

  // Calculate derivative (rate of change of error)
  // Note: Uses error from 10 samples ago (dt_window_size) to smooth out the derivative
  // term and reduce noise sensitivity. odt is 1/dt10.
  float derivative = (error - pid.last_errors[dt_index]) * odt;

  // Store error for next derivative calculation
  pid.last_errors[dt_index] = error;

  // Compute PID output: proportional + integral + derivative terms
  return pid.kp * error + pid.ki * pid.integral + pid.kd * derivative;
}

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
void admittance_step(float force, float dt10) {

  // 1. Calculate the "Spring" and "External" forces first (excluding damping)
  float spring_force = msd.k * (msd.virt_pos - msd.spring_zero);
  float acc_no_damping = (force - spring_force) * msd.inv_m;

  // 2. Update Velocity (Implicitly)
  // v_new = (v_old + dt * a_no_damping) / denom
  msd.virt_vel = (msd.virt_vel + acc_no_damping * dt10) / msd.denom;

  // Safety Clamp for Velocity
  msd.virt_vel = constrain(msd.virt_vel, -2.0, 2.0);

  // 3. Update Position (Standard Semi-Implicit)
  msd.virt_pos += msd.virt_vel * dt10;

  // Safety Clamp for Position
  msd.virt_pos = constrain(msd.virt_pos, lower_lim + 0.5, upper_lim);

  // Set global desired position
  des = msd.virt_pos;
}


// ============================================================================
// Force Sensor Reading
// ============================================================================

/**
 * Read and process force from a Wheatstone bridge
 * 
 * @param pair  Sensor pair index (0-4)
 * @return      Filtered force reading in Newtons
 */
float read_fsr(int pair) {
  // Read both sensors in the differential pair
  int read = analogRead(apins[pair * 2]);       // Left sensor
  int read2 = analogRead(apins[pair * 2 + 1]);  // Right sensor

  int pN = read - read2;
  int s = -12 * pN + 5582;
  // Subtract baseline (tare) to get relative force
  int current = s - base_foot[pair];
  int clean = median_step(&medians[pair], current);
  float avg = clean;  // Return filtered average

  // Convert grams to Newtons
  avg *= 0.0098066500286389;
  return avg;
}

/**
 * Calibrate/Tare a Wheatstone bridge
 * 
 * Reads the pin pair and updates a running average to establish
 * the baseline zero-force value.
 * 
 * @param pair  Sensor pair index (0-4)
 * @return      Current average baseline value
 */
int calibrate_fsr(int pair) {
  // Read both sensors in the differential pair
  int read = analogRead(apins[pair * 2]);       // Left sensor
  int read2 = analogRead(apins[pair * 2 + 1]);  // Right sensor

  int pN = read - read2;

  movavg_sum[pair] += pN;
  movavg_vals[pair] += 1;


  int avg = movavg_sum[pair] / movavg_vals[pair];
  return -12 * avg + 5582;
}

/**
 * Calculate net torque/force from all sensors
 * 
 * Computes a weighted moment about the foot's pivot point.
 * Front sensors (0-2) and back sensors (3-4) create opposing torques.
 * 
 * @return  Net torque [Nm]
 */
float force() {
  // Average force from front sensors (3 sensors)
  float top_avg = (new_foot[0] + new_foot[1] + new_foot[2]) * 0.33 * help_factor;

  // Average force from back sensors (2 sensors)
  float bot_avg = (new_foot[3] + new_foot[4]) * 0.5 * help_factor;

  // Moment arm lengths [m]
  float len_top = 0.11;  // Front sensor distance from pivot
  float len_bot = 0.07;  // Back sensor distance from pivot

  // Calculate the net force immediately
  // Positive pushes back, Negative pushes front
  float net_force = (top_avg * len_top) - (bot_avg * len_bot);

  return net_force;
}


// ============================================================================
// Debug Logging
// ============================================================================

/**
 * Capture system state for logging
 *
 * Stores the current state variables into the global log structure.
 * This snapshots the data so it can be printed incrementally.
 *
 * @param rad  Current position [rad]
 * @param f    Calculated net force/torque [Nm]
 */
void capture_log(float rad, float f) {
  last_log.time = millis();
  last_log.avg_loop = (millis() * 1000.0) / (loop_counter);
  last_log.top1 = new_foot[0];
  last_log.top2 = new_foot[1];
  last_log.top3 = new_foot[2];
  last_log.bot1 = new_foot[3];
  last_log.bot2 = new_foot[4];
  last_log.virt_pos = msd.virt_pos;
  last_log.virt_vel = msd.virt_vel;
  last_log.rad = rad;
  last_log.control_out = speed;
  last_log.f = f;
  last_log.index = 0;
}

/**
 * Print one field of the log entry
 *
 * To minimize loop blocking, this function prints only one data field per call.
 * It cycles through the fields based on last_log.index.
 */
void log() {
  switch (last_log.index) {
    case 0:
      Serial.print("time: ");
      Serial.print(last_log.time);
      break;
    case 1:
      Serial.print(", rad(rad): ");
      Serial.print(last_log.rad);
      break;
    case 2:
      Serial.print(", vpos(rad): ");
      Serial.print(last_log.virt_pos);
      break;
    case 3:
      Serial.print(", control_out: ");
      Serial.print(last_log.control_out);
      break;
    // case 2:
    //   Serial.print(", top1(N): ");
    //   Serial.print(last_log.top1);
    //   break;
    // case 3:
    //   Serial.print(", top2(N): ");
    //   Serial.print(last_log.top2);
    //   break;
    // case 4:
    //   Serial.print(", top3(N): ");
    //   Serial.print(last_log.top3);
    //   break;
    // case 5:
    //   Serial.print(", bot1(N): ");
    //   Serial.print(last_log.bot1);
    //   break;
    // case 6:
    //   Serial.print(", bot2(N): ");
    //   Serial.print(last_log.bot2);
    //   break;
    // case 7:
    //   Serial.print(", f(NM): ");
    //   Serial.print(last_log.f);
    //   break;
    // case 8:
    //   Serial.print(", vpos(rad): ");
    //   Serial.print(last_log.virt_pos);
    //   break;
    // case 9:
    //   Serial.print(", vvel(rad/s): ");
    //   Serial.print(last_log.virt_vel);
    //   break;
    // case 10:
    //   Serial.print(", avg_loop(us): ");
    //   Serial.print(last_log.avg_loop);
    //   break;
    // case 11:
    //   Serial.print(", control_out: ");
    //   Serial.print(last_log.control_out);
    //   break;
    default:
      break;
  }
  last_log.index += 1;
}
