typedef struct  {
  float kp;
  float ki;
  float kd;
  float integral;
  float last_error;
  float output;
} pid;

typedef struct  {
  float k;
  float c;
  float m;
  float last_pos;
  float last_vel;
  float output;
} admittance;

admittance m_msd;

float des;
int speed;
int p;
pid m_pid;

int apins[] = {A1, A2, A3, A4, A5, A6, A7, A8, A9, A10}; 
float basefoot[] = {0.0, 0.0, 0.0, 0.0, 0.0};
float newfoot[]  = {0.0, 0.0, 0.0, 0.0, 0.0};

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  pinMode(A0, INPUT);

  pinMode(A1, INPUT);
  pinMode(A2, INPUT);
  pinMode(A3, INPUT);
  pinMode(A4, INPUT);
  pinMode(A5, INPUT);
  pinMode(A6, INPUT);
  pinMode(A7, INPUT);
  pinMode(A8, INPUT);
  pinMode(A9, INPUT);
  pinMode(A10, INPUT);
  

  pinMode(8,OUTPUT);
  pinMode(9,OUTPUT);

  m_pid.kp = 130; // 130
  m_pid.ki = 0.135*m_pid.kp; // 0.135
  m_pid.kd = 1.3*m_pid.kp; // 1.3
  m_pid.integral = 0.0;
  m_pid.last_error = 0.0;
  m_pid.output = 0.0;

  m_msd.k = 0;
  m_msd.c = 0;
  m_msd.m = 0;
  m_msd.last_pos = analogRead(A0)*0.005+0.0565;
  m_msd.last_vel = 0;
  m_msd.output = 0;

  for (int i = 0; i < 5; i++){
    basefoot[i] = read_weat(i);
  }
}

void loop() {
  for (int i = 0; i < 5; i++){
    newfoot[i] = read_weat(i);
  }
  
  int read = analogRead(A0);
  if (read > 400 || read < 70) {
    speed = 0;
  } else {
    float rad = read*0.005+0.0565;
    float err = des - rad;
    pidstep(&m_pid, err, 0.1);
    speed = m_pid.output;
    if (p > 500 ){
      p = 0;
      Serial.print("top: ");
      Serial.print(newfoot[0]);
      Serial.print(", ");
      Serial.print(newfoot[1]);
      Serial.print(", ");
      Serial.print(newfoot[2]);
      Serial.print("; but: ");
      Serial.print(newfoot[3]);
      Serial.print(", ");
      Serial.print(newfoot[4]);
      Serial.print("; rad: ");
      Serial.print(rad);
      Serial.print("; ");
      Serial.print(err);
      Serial.print("; integral: ");
      Serial.print(m_pid.integral);
      Serial.print("; con: ");
      Serial.println(speed);
    }
    if (speed > 30){
      speed = 30;
    } else if (speed < -30) {
      speed = -30;
    }
    p++;
  }
  if (speed == 0){
    analogWrite(8,0);
    analogWrite(9,0);
  } else if (speed < 0 ){
    analogWrite(9,abs(speed));
    analogWrite(8,0);
  } else {
    analogWrite(9,0);
    analogWrite(8,abs(speed));
  }
}

void pidstep(pid *m_pid, float error, float dt) {
  m_pid->integral += error * dt;
  if (m_pid->integral > 0.5){
    m_pid->integral = 0.5;
  } else if (m_pid->integral < -0.5) {
    m_pid->integral = -0.5;
  }

  float derivative = (error - m_pid->last_error) / dt;

  m_pid->output = m_pid->kp * error + m_pid->ki * m_pid->integral + m_pid->kd * derivative;

  m_pid->last_error = error;
}

float read_weat(int pair) {
  int read = analogRead(apins[pair *2]);
  int read2 = analogRead(apins[pair *2 + 1]);

    // convert to volts
  float Vleft = read * (5.0 / 1023.0);
  float Vright = read2 * (5.0 / 1023.0);

  float Vdiff = Vleft - Vright;  

  const float a = -15.0;
  const float offset = 0.0;
  float N = a * (Vdiff - offset);
  return abs(N - basefoot[pair]);

}
void admittance (float force , float pos) {
  float vel = m_msd.last_pos - pos;
  float acc = m_msd.last_vel - vel;
  m_msd.output = (-m_msd.c*vel-m_msd.m*acc + force)/m_msd.k;
  return 
  
}
