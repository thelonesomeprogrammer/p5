typedef struct {
  float kp;
  float ki;
  float kd;
  float integral;
  float last_error;
  float output;
} pid;

int des;
int speed;
pid m_pid;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(A0, INPUT);
  pinMode(8,OUTPUT);
  pinMode(9,OUTPUT);
  m_pid.kp = 80;
  m_pid.ki = 2*m_pid.kp;
  m_pid.kd = 0.2*m_pid.kp;
  m_pid.integral = 0.0;
  m_pid.last_error = 0.0;
  m_pid.output = 0.0;
  des = 1;
}

void loop() {
  delay(10);

  int read = analogRead(A0);
  if (read > 400) {
    Serial.println("fuck");
    speed = 0;
  } else if (read < 70) {
    Serial.println("pussy");
    speed = 0;
  } else {
    float rad = read*0.005+0.0565;
    Serial.println(rad);
    float err = des - rad;
    pidstep(&m_pid, err, 0.1);
    speed = m_pid.output;
  }
  if (speed == 0){
    analogWrite(8,0);
    analogWrite(9,0);
  } else if (speed < 0 ){
    analogWrite(8,0);
    analogWrite(9,30);
  } else {
    analogWrite(8,30);
    analogWrite(9,0);
  }
}

void pidstep(pid *m_pid, float error, float dt) {

m_pid->integral += error * dt;
float derivative = (error - m_pid->last_error) / dt;

m_pid->output = m_pid->kp * error + m_pid->ki * m_pid->integral + m_pid->kd * derivative;

m_pid->last_error = error;
}
