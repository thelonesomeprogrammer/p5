const int A_LEFT = A0;
const int A_RIGHT = A1;
const int NUM_SAMPLES = 50;
const float VCC = 5.0;
const int ADC_MAX = 1023;
float firstMeasurement = 0.0;
bool firstTaken = false;
void setup() {
  Serial.begin(115200);
  analogReference(DEFAULT);  // 5V reference; change if you use INTERNAL ref
  delay(100);
}

float readAvg(int pin) {
  long sum = 0;
  for (int i = 0; i < NUM_SAMPLES; ++i) {
    sum += analogRead(pin);
  }
  return float(sum) / NUM_SAMPLES;
}

void loop() {
  float rawLeft = readAvg(A_LEFT);
  float rawRight = readAvg(A_RIGHT);

  // convert to volts
  float Vleft = rawLeft * (VCC / ADC_MAX);
  float Vright = rawRight * (VCC / ADC_MAX);

  float Vdiff = Vleft - Vright;  

  //old a
  //const float a = -15.0;
  //new a
  const float a = -15.0;  // example slope (N per V) — set from calibration
  const float offset = 0.0;
  float forceN = a * (Vdiff - offset);

  //new calibrated but with wrong diameter, need second biggest
  // const float a = 729601.6875;
  // const float c = -15.476274;
  //float forceN = a * pow(Vdiff, c);

  if (!firstTaken) {
    firstMeasurement = forceN;
    firstTaken = true;
    Serial.println("First measurement stored.");
  } else {
    float difference = abs(forceN - firstMeasurement);
    Serial.print(firstMeasurement);

    Serial.print("Vleft: ");
    Serial.print(Vleft, 4);
    Serial.print("  Vright: ");
    Serial.print(Vright, 4);
    Serial.print("  Vdiff: ");
    Serial.print(Vdiff, 4);
    Serial.print("  Force (est): ");
    Serial.print(difference, 3);
    Serial.println(" N");
    delay(200);
  }
}