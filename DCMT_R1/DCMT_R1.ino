#include <Wire.h>
#include <FastLED.h>
#include <PID_v1.h>

#define I2C_ADR 23

#define ESTOP   2
#define LED_PIN 5

#define DIR1  7
#define MC1   6
#define BR1   8
#define DIR2  10
#define MC2   11
#define BR2   12

#define ENC_A1  A3
#define ENC_B1  A2
#define ENC_A2  A1
#define ENC_B2  A0

#define SERIAL_UPDATE_TIME_MS       1000
#define PUMP_PULSE_PERIOD_MS        2000
#define MOTOR_MAX_SPEED             255
#define MOTOR_MIN_SPEED             130
#define TURBIDITY_PUMP_TIME_MS      10000   // time to cycle new sample through tubes
#define TURBIDITY_PUMP_TIME_R_MS    10000   // time to reverse cycle sample
#define TURBIDITY_PUMP_SPEED        60      // speed of turbidity pump (percent)
#define TURBIDITY_SAMPLE_WAIT_MS    3000    // wait time for bubbles to settle before measurement
#define TURBIDITY_OFFSET            0       // tap water should be approx. 0 NTU

CRGB led;
#define NUM_TURB_LEDS   20
CRGB turbLEDs[NUM_TURB_LEDS];

union INTUNION_t //Define an int that can be broken up and sent via I2C
{
  int number;
  uint8_t bytes[4];
};

union FLOATUNION_t //Define an int that can be broken up and sent via I2C
{
  float number;
  uint8_t bytes[4];
};

struct DCMT_t {
  int m1Speed = 0;   // motor 1 speed (0-255)
  int m2Speed = 0;   // motor 2 speed (0-255)
  bool m1Dir;  // motor 1 direction (0:backwards, 1:forwards)
  bool m2Dir;  // motor 2 direction (0:backwards, 1:forwards)
  bool m1En;   // motor 1 enable (0:off/apply brake, 1:on/coast)
  bool m2En;   // motor 2 enable (0:off/apply brake, 1:on/coast)
  double m1OnTime;   // for pulsing pump
  double m2OnTime;
  double pHSetpoint;
  double currentPH;
  double Kp_pH;
  double Ki_pH;
  double Kd_pH;
  bool pulsePump = false;   // by default, DCMT acts as motor controller
  bool turbPump1 = false;    // for enabling measuring turbitidy from motor 1
  bool turbPump2 = false;    // for enabling measuring turbitidy from motor 2
  float turbVoltage[2];
  float currentTurbidity[2];
  uint32_t turbSamplePeriod1 = 10000; // Sampling time is 10s by default
  uint32_t turbSamplePeriod2 = 10000;
} DCMT, DCMT_old;

// Acid control activates when pH > Setpoint so control needs to be reversed
PID basePID(&(DCMT.currentPH), &(DCMT.m1OnTime), &(DCMT.pHSetpoint), DCMT.Kp_pH, DCMT.Ki_pH, DCMT.Kd_pH, DIRECT);
PID acidPID(&(DCMT.currentPH), &(DCMT.m2OnTime), &(DCMT.pHSetpoint), DCMT.Kp_pH, DCMT.Ki_pH, DCMT.Kd_pH, REVERSE);

// for tracking pulse periods
unsigned long m1StartTime;
unsigned long m2StartTime;

long lastSerialPrint = 0;
long lastTurbiditySequence1 = 0;
int turbidityCycle1 = 0;    // turbidity cycle sequence indicator
long lastTurbiditySequence2 = 0;
int turbidityCycle2 = 0;    // turbidity cycle sequence indicator

bool E_STOP = false;

void estop()
{
  if(digitalRead(ESTOP) == HIGH)    // when set to HIGH state
  {
    led = CRGB::Red;
    FastLED.show();

    // save states
    DCMT_old.m1Speed = DCMT.m1Speed;
    DCMT_old.m2Speed = DCMT.m2Speed;

    // set critical states to zero
    DCMT.m1Speed = 0;
    DCMT.m2Speed = 0;
    // disable motors
    DCMT.m1En = false;
    DCMT.m2En = false;

    E_STOP = true;
    Serial.println("ESTOP PRESSED!");
  }
  else    // when set to LOW state
  {
    led = CRGB::Black;
    FastLED.show();

    // reassign old states if they haven't changed
    if(!DCMT.m1Speed)
      DCMT.m1Speed = DCMT_old.m1Speed;
    if(!DCMT.m2Speed)
      DCMT.m2Speed = DCMT_old.m2Speed;

    DCMT.m1En = true;
    DCMT.m2En = true;

    E_STOP = false;

    Serial.println("ESTOP RELEASED!");
  }
  actuateMotors();
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  FastLED.addLeds<NEOPIXEL, LED_PIN>(&led, 1);
  //FastLED.addLeds<NEOPIXEL, ENC_A2>(turbLEDs, NUM_TURB_LEDS);

  pinMode(ESTOP, INPUT);
  attachInterrupt(digitalPinToInterrupt(ESTOP), estop, CHANGE);

  pinMode(BR1, INPUT_PULLUP);
  pinMode(BR2, INPUT_PULLUP);
  pinMode(MC1, OUTPUT);
  pinMode(MC2, OUTPUT);
  pinMode(DIR1, OUTPUT);
  pinMode(DIR2, OUTPUT);
  pinMode(ENC_B1, INPUT);   // turbidity sense 1
  pinMode(ENC_B2, INPUT);   // turbidity sense 2

  // motor 1
  digitalWrite(DIR1, HIGH);
  digitalWrite(MC1, LOW);
  // motor 2
  digitalWrite(DIR2, HIGH);
  digitalWrite(MC2, LOW);

  pinMode(BR1, OUTPUT);
  pinMode(BR2, OUTPUT);

  Wire.begin(I2C_ADR);
  Wire.onReceive(receiveEvent);
  Wire.onRequest(requestEvent);

  // set window limits for PID (half of maximum period)
  acidPID.SetOutputLimits(0, PUMP_PULSE_PERIOD_MS/2);
  basePID.SetOutputLimits(0, PUMP_PULSE_PERIOD_MS/2);

  // turn on PID
  acidPID.SetMode(AUTOMATIC);
  basePID.SetMode(AUTOMATIC);

  // enable turbidity LEDs
  for(int i=0;i<20;i++)
    turbLEDs[i] = CRGB::White;
  FastLED.show();

  delay(2000);

  m1StartTime = millis();
  m2StartTime = millis();
}

void loop() {
  if(!E_STOP){
    if(DCMT.pulsePump && DCMT.pHSetpoint != 0){   // run as pH controller
      setPIDTunings();
      if(DCMT.currentPH >= DCMT.pHSetpoint){
        // turn off base pump and prevent basePID from updating
        DCMT.m1OnTime = 0;
        basePID.SetMode(MANUAL);
        acidPID.SetMode(AUTOMATIC);
        acidPID.Compute();  // acid control when pH > setpoint
      }
      else{
        // turn off acid pump and prevent acidPID from updating
        DCMT.m2OnTime = 0;
        acidPID.SetMode(MANUAL);
        basePID.SetMode(AUTOMATIC);
        basePID.Compute();   // base control when pH < setpoint
      }
      actuatePumps();
    }else if(DCMT.pulsePump && DCMT.pHSetpoint == 0){
      DCMT.m1Speed = 0;
      DCMT.m2Speed = 0;   // set motor speeds to 0
      DCMT.m1OnTime = 0;
      DCMT.m2OnTime = 0;  // reset PID
      basePID.SetMode(MANUAL);
      acidPID.SetMode(MANUAL);
      actuateMotors();
    }
    else if(DCMT.turbPump1 || DCMT.turbPump2){   // run as turbidity pump
      measureTurbidity();
    }else{      // run as continuous motor/pump
      actuateMotors();
    }
  }
  printOutput();
}

void measureTurbidity()
{
  if(DCMT.turbPump1){
    if(turbidityCycle1 == 0 && millis() - lastTurbiditySequence1 >= (DCMT.turbSamplePeriod1 - TURBIDITY_PUMP_TIME_MS - TURBIDITY_SAMPLE_WAIT_MS - TURBIDITY_PUMP_TIME_R_MS))    // wait until its time to take a new sample
    {
      lastTurbiditySequence1 = millis();
      turbidityCycle1++;
      DCMT.m1Dir = 1;
      DCMT.m1Speed = TURBIDITY_PUMP_SPEED * 255./100.;
      Serial.println("Cycling new sample for pump 1...");
      actuateMotors();
    }
    if(turbidityCycle1 == 1 && (millis() - lastTurbiditySequence1 >= TURBIDITY_PUMP_TIME_MS))    // cycle new sample for some time
    {
      lastTurbiditySequence1 = millis();
      turbidityCycle1++;
      DCMT.m1Speed = 0;   // stop pump until next sample
      actuateMotors();
    }
    if(turbidityCycle1 == 2 && (millis() - lastTurbiditySequence1 >= TURBIDITY_SAMPLE_WAIT_MS)) // wait until bubbles settle and measure
    {
      lastTurbiditySequence1 = millis();
      turbidityCycle1++;
      DCMT.turbVoltage[0] = analogRead(ENC_B1)*5.0/1023;
      Serial.println("Sample 1 measured!");

      DCMT.m1Dir = 0;
      DCMT.m1Speed = TURBIDITY_PUMP_SPEED * 255./100.;
      actuateMotors();
      Serial.println("Reversing sample 1...");
    }
    if(turbidityCycle1 == 3 && (millis() - lastTurbiditySequence1 >= TURBIDITY_PUMP_TIME_R_MS))    // reverse sample for some time
    {
      lastTurbiditySequence1 = millis();
      turbidityCycle1 = 0;   // back to starting sequence (waiting)
      DCMT.m1Speed = 0;   // stop pump until next sample
      actuateMotors();
    }
  }
  if(DCMT.turbPump2){
    if(turbidityCycle2 == 0 && millis() - lastTurbiditySequence2 >= (DCMT.turbSamplePeriod2 - TURBIDITY_PUMP_TIME_MS - TURBIDITY_SAMPLE_WAIT_MS - TURBIDITY_PUMP_TIME_R_MS))    // wait until its time to take a new sample
    {
      lastTurbiditySequence2 = millis();
      turbidityCycle2++;
      DCMT.m2Dir = 1;
      DCMT.m2Speed = TURBIDITY_PUMP_SPEED * 255./100.;
      Serial.println("Cycling new sample for pump 2...");
      actuateMotors();
    }
    if(turbidityCycle2 == 1 && (millis() - lastTurbiditySequence2 >= TURBIDITY_PUMP_TIME_MS))    // cycle new sample for some time
    {
      lastTurbiditySequence2 = millis();
      turbidityCycle2++;
      DCMT.m2Speed = 0;   // stop pump until next sample
      actuateMotors();
    }
    if(turbidityCycle2 == 2 && (millis() - lastTurbiditySequence2 >= TURBIDITY_SAMPLE_WAIT_MS)) // wait until bubbles settle and measure
    {
      lastTurbiditySequence2 = millis();
      turbidityCycle2++;
      DCMT.turbVoltage[1] = analogRead(ENC_B2)*5.0/1023;
      Serial.println("Sample 2 measured!");

      DCMT.m2Dir = 0;
      DCMT.m2Speed = TURBIDITY_PUMP_SPEED * 255./100.;
      actuateMotors();
      Serial.println("Reversing sample 2...");
    }
    if(turbidityCycle2 == 3 && (millis() - lastTurbiditySequence2 >= TURBIDITY_PUMP_TIME_R_MS))    // reverse sample for some time
    {
      lastTurbiditySequence2 = millis();
      turbidityCycle2 = 0;   // back to starting sequence (waiting)
      DCMT.m2Speed = 0;   // stop pump until next sample
      actuateMotors();
    }
  }
}

// need to change once biomass is calibrated
void calculateTurbidity(){
  float x;
  if(DCMT.turbVoltage[1] >= 2.5){
    x = DCMT.turbVoltage[1];
  }else{
    x = 2.5;
  }
  DCMT.currentTurbidity[1] = (5742.3 - 1120.4*x) * x - 4352.9 - TURBIDITY_OFFSET;
}

void setPIDTunings()
{
  acidPID.SetTunings(DCMT.Kp_pH, DCMT.Ki_pH, DCMT.Kd_pH);
  basePID.SetTunings(DCMT.Kp_pH, DCMT.Ki_pH, DCMT.Kd_pH);
}

void actuatePumps()
{
  // reset states when pH is set to zero
  if(DCMT.pHSetpoint == 0){
    DCMT.m1Speed = 0;
    DCMT.m2Speed = 0;

    DCMT.m1OnTime = 0;
    DCMT.m2OnTime = 0;
  }
  // Motor 1 (Acid Pump)
  if (millis() - m1StartTime > PUMP_PULSE_PERIOD_MS)
  { //time to shift the Relay Window
    m1StartTime += PUMP_PULSE_PERIOD_MS;
  }
  if ((int)(DCMT.m1OnTime) > millis() - m1StartTime)
  {
    digitalWrite(BR1, !DCMT.m1En);    // apply brake if enable is FALSE
    digitalWrite(DIR1, DCMT.m1Dir);
    analogWrite(MC1, DCMT.m1Speed);
  }
  else digitalWrite(BR1, HIGH);   // set brake to stop motor

  // Motor 2 (Base Pump)
  if (millis() - m2StartTime > PUMP_PULSE_PERIOD_MS)
  { //time to shift the Relay Window
    m2StartTime += PUMP_PULSE_PERIOD_MS;
  }
  if ((int)(DCMT.m2OnTime) > millis() - m2StartTime)
  {
    digitalWrite(BR2, !DCMT.m2En);    // apply brake if enable is FALSE
    digitalWrite(DIR2, DCMT.m2Dir);
    analogWrite(MC2, DCMT.m2Speed);
  }
  else digitalWrite(BR2, HIGH);   // set brake to stop motor
}

void printOutput()
{
  if(millis() - lastSerialPrint >= SERIAL_UPDATE_TIME_MS)
  {
    Serial.print(analogRead(ENC_B1));
    Serial.print(", ");
    Serial.print(analogRead(ENC_B2));
    /*
    if(DCMT.pulsePump){
      Serial.print("PID: ");
      Serial.print(DCMT.Kp_pH);
      Serial.print(",");
      Serial.print(DCMT.Ki_pH);
      Serial.print(",");
      Serial.print(DCMT.Kd_pH);

      Serial.print("\tpH: ");
      Serial.print(DCMT.currentPH);
      Serial.print("\tSetpoint: ");
      Serial.print(DCMT.pHSetpoint);
      Serial.print("\tonTime1: ");
      Serial.print((int)(DCMT.m1OnTime));
      Serial.print("\tonTime2: ");
      Serial.print((int)(DCMT.m2OnTime));
    }else{
      Serial.print("Motor 1 Speed: ");
      Serial.print(DCMT.m1Speed);
      Serial.print("\tDirection: ");
      Serial.print(DCMT.m1Dir);
      Serial.print("\tMotor 2 Speed: ");
      Serial.print(DCMT.m2Speed);
      Serial.print("\tDirection: ");
      Serial.print(DCMT.m2Dir);
      if(DCMT.turbPump1 || DCMT.turbPump2){
        Serial.print("\tSampleTime1: ");
        Serial.print(DCMT.turbSamplePeriod1);
        Serial.print("\tSampleTime2: ");
        Serial.print(DCMT.turbSamplePeriod2);
        Serial.print("\tTurbidity1: ");
        Serial.print(DCMT.turbVoltage[0]);
        Serial.print("\tTurbidity2: ");
        Serial.print(DCMT.turbVoltage[1]);
      }
    }
    */
    Serial.println();
    lastSerialPrint = millis();
  }
}

void actuateMotors()
{
  // motor 1
  digitalWrite(BR1, !DCMT.m1En);    // apply brake if enable is FALSE
  digitalWrite(DIR1, DCMT.m1Dir);
  analogWrite(MC1, DCMT.m1Speed);
    
  // motor 2
  digitalWrite(BR2, !DCMT.m2En);    // apply brake if enable is FALSE
  digitalWrite(DIR2, DCMT.m2Dir);
  analogWrite(MC2, DCMT.m2Speed);
}

void requestEvent() {
  FLOATUNION_t turb1, turb2;

  if(isnan(DCMT.turbVoltage[0]) || !(DCMT.turbPump1))
    turb1.number = 0;
  else
    turb1.number = DCMT.turbVoltage[0];

  if(isnan(DCMT.turbVoltage[1]) || !(DCMT.turbPump2))
    turb2.number = 0;
  else
    turb2.number = DCMT.turbVoltage[1];

  for (int i = 0; i < 4; i++) Wire.write(turb1.bytes[i]);
  for (int i = 0; i < 4; i++) Wire.write(turb2.bytes[i]);
}

void receiveEvent(int howMany) 
{
  byte in_char;
  char in_data[20];

  led = CRGB::Green;
  FastLED.show();

  //Serial.println("Receiving: ");
  int i=0;
  while(Wire.available())
  {
    in_char = Wire.read();
    in_data[i] = in_char;
    Serial.print((byte)in_data[i]);
    Serial.print(",");
    i++;
  }
  Serial.println();

  setParametersDCMT(in_data);

  led = CRGB::Black;
  FastLED.show();
}

void setParametersDCMT(char *in_data)
{
  INTUNION_t int1, int2, int3;

  FLOATUNION_t float1, float2, float3;

  switch(in_data[0])
  {
    case 'M':   // motor edit, disable PID
      DCMT.pulsePump = false;    // disable pulse behavior

      //DCMT.m1Speed = 0;
      //DCMT.m2Speed = 0;   // set motor speeds to 0 to reset from PID
      if(in_data[1] == 1){   // motor 1
        DCMT.turbPump1 = false;
        for(int i=0;i<4;i++) int1.bytes[i] = in_data[i+2];
        if(int1.number >= 0){
          DCMT.m1Speed = map(int1.number, 0, 100, 0, 255);
          DCMT.m1Dir = 1;
        }else{
          DCMT.m1Speed = map(0-int1.number, 0, 100, 0, 255);
          DCMT.m1Dir = 0;
        }
        if(!E_STOP)
          DCMT.m1En = in_data[6];
      }
      if(in_data[1] == 2){   // motor 2
        DCMT.turbPump2 = false;
        for(int i=0;i<4;i++) int1.bytes[i] = in_data[i+2];
        if(int1.number >= 0){
          DCMT.m2Speed = map(int1.number, 0, 100, 0, 255);
          DCMT.m2Dir = 1;
        }else{
          DCMT.m2Speed = map(0-int1.number, 0, 100, 0, 255);
          DCMT.m2Dir = 0;
        }
        if(!E_STOP)
          DCMT.m2En = in_data[6];
      }
      break;
    
    case 'p':   // pH setpoint
      DCMT.pulsePump = true;
      DCMT.turbPump1 = false;
      DCMT.turbPump2 = false;

      for(int i=0;i<4;i++) float1.bytes[i] = in_data[i+1];
      for(int i=0;i<4;i++) float2.bytes[i] = in_data[i+5];

      DCMT.currentPH = (double)(float1.number);
      DCMT.pHSetpoint = (double)(float2.number);
      
      if(!E_STOP){
        DCMT.m1En = true;
        DCMT.m2En = true;
      }
      DCMT.m1Dir = 1;
      DCMT.m2Dir = 1;
      DCMT.m1Speed = MOTOR_MIN_SPEED;
      DCMT.m2Speed = MOTOR_MIN_SPEED;
      break;

    case 'P': // populate DCMT params, set motor to min speed, forward, enable PID
      DCMT.pulsePump = true;
      DCMT.turbPump1 = false;
      DCMT.turbPump2 = false;

      for(int i=0;i<4;i++) float1.bytes[i] = in_data[i+1];
      for(int i=0;i<4;i++) float2.bytes[i] = in_data[i+5];
      for(int i=0;i<4;i++) float3.bytes[i] = in_data[i+9];

      DCMT.Kp_pH = (double)(float1.number);
      DCMT.Ki_pH = (double)(float2.number);
      DCMT.Kd_pH = (double)(float3.number);

      if(!E_STOP){
        DCMT.m1En = true;
        DCMT.m2En = true;
      }
      DCMT.m1Dir = 1;
      DCMT.m2Dir = 1;
      DCMT.m1Speed = MOTOR_MIN_SPEED;
      DCMT.m2Speed = MOTOR_MIN_SPEED;
      break;

// T, motor (1,2), direction, enable, sample period (s)
    case 'T':
      DCMT.pulsePump = false;    // disable pulse behavior

      for(int i=0;i<4;i++) int1.bytes[i] = (uint8_t)in_data[i+4];

      if(in_data[1] == 1){   // motor 1
        DCMT.turbSamplePeriod1 = (uint32_t)int1.number * 1000;    // assign sample period
        DCMT.turbPump1 = true;
        DCMT.m1Dir = in_data[2];
        if(!E_STOP)
          DCMT.m1En = in_data[3];
      }
      if(in_data[1] == 2){   // motor 2
        DCMT.turbSamplePeriod2 = (uint32_t)int1.number * 1000;    // assign sample period
        DCMT.turbPump2 = true;
        DCMT.m2Dir = in_data[2];
        if(!E_STOP)
          DCMT.m2En = in_data[3];
      }
      break;
  }
}
