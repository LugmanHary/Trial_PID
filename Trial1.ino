//Library
#include <Keypad.h>
#include <Wire.h>
#include <EEPROM.h>
#include <LiquidCrystal_I2C.h>
#include <DHT.h>

//Liquid Crystal Display with I2C
const int rs = 8, en = 9, d4 = 10, d5 = 11, d6 = 12, d7 = 13;
LiquidCrystal_I2C lcd(0x27, 16, 2);

//Pin I/O Arduino
const int pwm_fan = 11;            //pwm fan pin
#define DHTPIN A0          //dht pin
#define DHTTYPE DHT11      //type
#define IN1 12             //deklarasi pin IN1
#define IN2 13             //deklarasi pin IN2
DHT dht(DHTPIN, DHTTYPE);

//Keypad
char Key;
const byte ROWS = 4;
const byte COLS = 4;

char keys[ROWS][COLS] = {
  {'D', 'C', 'B', 'A'},
  {'#', '9', '6', '3'},
  {'0', '8', '5', '2'},
  {'*', '7', '4', '1'}
};

byte rowPins[ROWS] = {3, 4, 5, 6};
byte colPins[COLS] = {7, 8, 9, 10};
Keypad myKeypad = Keypad( makeKeymap(keys), rowPins, colPins, ROWS, COLS);

//Variabel Data
float SV, PV, PVf, PVf_1, a, fc, RC, Ts;
unsigned long timeold; //variabel tipe data long
int temp, tempfix;
int x;

//Variabel PID
float kp, nilai_kp;
float ki, nilai_ki;
float kd, nilai_kd;
float sp, nilai_sp;
float Error, ErrorX, Summer, Diff;
int p, i, d, pid, out;

//IR FC-03
const byte PulsesPerRevolution = 2;
const unsigned long ZeroTimeout = 100000;
const byte numReadings = 2;

volatile unsigned long LastTimeWeMeasured;
volatile unsigned long PeriodBetweenPulses = ZeroTimeout + 1000;
volatile unsigned long PeriodAverage = ZeroTimeout + 1000;
unsigned long FrequencyRaw;
unsigned long FrequencyReal;
unsigned long RPM;
unsigned int PulseCounter = 1;
unsigned long PeriodSum;

unsigned long LastTimeCycleMeasure = LastTimeWeMeasured;
unsigned long CurrentMicros = micros();
unsigned int AmountOfReadings = 1;
unsigned int ZeroDebouncingExtra;
unsigned long readings[numReadings];
unsigned long readIndex;
unsigned long total;
unsigned long average;

void setup() {

  //Setting PIN
  Serial.begin(9600);
  dht.begin();
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(pwm_fan, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(2), Pulse_Event, RISING);

  //Kalibrasi
  fc = 1.256;
  RC = 1 / (6.28 - *fc);
  Ts = 0.001;
  a = RC / 0.01;
  PVf_1 = 0;
  timeold = 0;

  //Turn LCD on
  lcd.backlight();
  lcd.init();
  lcd.begin(16, 2);
}

void set_kp() {
  lcd.setCursor(0, 0);
  lcd.print("SET KP = ");

  Key = myKeypad.getKey();

  if (Key >= '0' && Key <= '9')
  {
    kp = kp * 10 + (Key - '0');
    lcd.setCursor(0, 1);
    lcd.print(kp);

  }

  if (Key == 'A') {
    lcd.clear();
    delay(1000);
    kp = kp / 1;
    nilai_kp = kp;
    return;
  }

  if (Key == 'B') {
    lcd.clear();
    delay(1000);
    kp = kp / 10;
    nilai_kp = kp;
    return;
  }

  if (Key == 'C') {
    lcd.clear();
    delay(1000);
    kp = kp / 100;
    nilai_kp = kp;
    return;
  }

  if (Key == 'D') {
    lcd.clear();
    delay(1000);
    kp = kp / 1000;
    nilai_kp = kp;
    return;
  }

  if (Key == '*') {
    lcd.clear();
    delay(1000);
    return;
  }
  if (Key == "#") {
    lcd.clear();
    x = 0;
    kp, nilai_kp = 0;
    ki, nilai_ki = 0;
    kd, nilai_kd = 0;
    sp, nilai_sp = 0;
    pid = 0;
    delay(100);
    return;
  }
  set_kp();
}

void set_ki() {
  lcd.setCursor(0, 0);
  lcd.print("SET KI = ");

  Key = myKeypad.getKey();

  if (Key >= '0' && Key <= '9')
  {
    ki = ki * 10 + (Key - '0');
    lcd.setCursor(0, 1);
    lcd.print(ki);

  }

  if (Key == 'A') {
    lcd.clear();
    delay(1000);
    ki = ki / 1;
    nilai_ki = ki;
    return;
  }

  if (Key == 'B') {
    lcd.clear();
    delay(1000);
    ki = ki / 10;
    nilai_ki = ki;
    return;
  }

  if (Key == 'C') {
    lcd.clear();
    delay(1000);
    ki = ki / 100;
    nilai_ki = ki;
    return;
  }
  if (Key == 'D') {
    lcd.clear();
    delay(1000);
    ki = ki / 1000;
    nilai_ki = ki;
    return;
  }

  if (Key == '*') {
    lcd.clear();
    delay(1000);
    return;
  }
  if (Key == "#") {
    lcd.clear();
    x = 0;
    kp, nilai_kp = 0;
    ki, nilai_ki = 0;
    kd, nilai_kd = 0;
    sp, nilai_sp = 0;
    pid = 0;
    delay(100);
    return;
  }
  set_ki();
}

void set_kd() {
  lcd.setCursor(0, 0);
  lcd.print("SET KD = ");

  Key = myKeypad.getKey();

  if (Key >= '0' && Key <= '9')
  {
    kd = kd * 10 + (Key - '0');
    lcd.setCursor(0, 1);
    lcd.print(kd);

  }

  if (Key == 'A') {
    lcd.clear();
    delay(1000);
    kd = kd / 1;
    nilai_kd = kd;
    return;
  }

  if (Key == 'B') {
    lcd.clear();
    delay(1000);
    kd = kd / 10;
    nilai_kd = kd;
    return;
  }

  if (Key == 'C') {
    lcd.clear();
    delay(1000);
    kd = kd / 100;
    nilai_kd = kd;
    return;
  }
  if (Key == 'D') {
    lcd.clear();
    delay(1000);
    kd = kd / 1000;
    nilai_kd = kd;
    return;
  }
  if (Key == '*') {
    lcd.clear();
    delay(1000);
    return;
  }
  if (Key == "#") {
    lcd.clear();
    x = 0;
    kp, nilai_kp = 0;
    ki, nilai_ki = 0;
    kd, nilai_kd = 0;
    sp, nilai_sp = 0;
    pid = 0;
    delay(100);
    return;
  }
  set_kd();
}

void set_sp() {
  lcd.setCursor(0, 0);
  lcd.print("SET SP = ");

  Key = myKeypad.getKey();

  if (Key >= '0' && Key <= '9')
  {
    sp = sp * 10 + (Key - '0');
    lcd.setCursor(0, 1);
    lcd.print(sp);

  }

  if (Key == 'A') {
    lcd.clear();
    delay(1000);
    sp = sp / 1;
    nilai_sp = sp;
    return;
  }

  if (Key == 'B') {
    lcd.clear();
    delay(1000);
    sp = sp / 10;
    nilai_sp = sp;
    return;
  }

  if (Key == 'C') {
    lcd.clear();
    delay(1000);
    sp = sp / 100;
    nilai_sp = sp;
    return;
  }
  if (Key == 'D') {
    lcd.clear();
    delay(1000);
    sp = sp / 1000;
    nilai_sp = sp;
    return;
  }

  if (Key == '*') {
    lcd.clear();
    delay(1000);
    return;
  }
  if (Key == "#") {
    lcd.clear();
    x = 0;
    kp, nilai_kp = 0;
    ki, nilai_ki = 0;
    kd, nilai_kd = 0;
    sp, nilai_sp = 0;
    pid = 0;
    delay(100);
    return;
  }
  set_sp();
}

void cekpid() {

  lcd.setCursor(0, 0);
  lcd.print("KP=");
  lcd.print(nilai_kp, 2);
  lcd.print(" KI=");
  lcd.print(nilai_ki, 2);

  lcd.setCursor(0, 1);
  lcd.print("KD=");
  lcd.print(nilai_kd, 2);
  lcd.print(" SP=");
  lcd.print(nilai_sp, 2);

  Key = myKeypad.getKey();

  if (Key == '*') {
    lcd.clear();
    delay(1000);
    return;
  }
  if (Key == "#") {
    lcd.clear();
    x = 0;
    kp, nilai_kp = 0;
    ki, nilai_ki = 0;
    kd, nilai_kd = 0;
    sp, nilai_sp = 0;
    pid = 0;
    delay(100);
    return;
  }
  cekpid();
}

void mulai() {

  //membaca suhu dgn filter
  float t = dht.readTemperature();
  temp = t;
  PVf = (temp + a * PVf_1) / (a + 1);
  tempfix = PVf;

  //motor
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(pwm_fan, out);

  //process PID
  Error = nilai_sp - temp;
  p = Error * nilai_kp;
  Summer = Error + ErrorX;
  i = nilai_ki * Summer;
  Diff = Error - ErrorX;
  d = nilai_kd * Diff;
  pid = p + i + d;
  pid = pid * -1;
  out = min(max(pid, 0), 255);

  if (pid < 0) {
    pid = 0;
  }
  timeold = millis();

  //menampilkan nilai pada lcd
  lcd.setCursor(0, 0);
  lcd.print("TMP=");
  lcd.print(tempfix);
  lcd.print("       ");
  lcd.setCursor(0, 1);
  lcd.print("PID=");
  lcd.print(out);
  lcd.print("       ");
  lcd.setCursor(9, 0);
  lcd.print("RPM=");
  lcd.print(RPM);
  lcd.print("       ");
  lcd.setCursor(9, 1);
  lcd.print("ERR=");
  lcd.print(Error);
  lcd.print("       ");

  //mengecek pulse sensor
  LastTimeCycleMeasure = LastTimeWeMeasured;
  CurrentMicros = micros();
  if (CurrentMicros < LastTimeCycleMeasure) {
    LastTimeCycleMeasure = CurrentMicros;
  }
  FrequencyRaw = 10000000000 / PeriodAverage;
  if (PeriodBetweenPulses > ZeroTimeout - ZeroDebouncingExtra || CurrentMicros - LastTimeCycleMeasure > ZeroTimeout - ZeroDebouncingExtra) {
    FrequencyRaw = 0;  // Set frequency as 0.
    ZeroDebouncingExtra = 2000;
  } else {
    ZeroDebouncingExtra = 0;
  }
  FrequencyReal = FrequencyRaw / 10000;

  RPM = FrequencyRaw / PulsesPerRevolution * 60;
  RPM = RPM / 10000;
  total = total - readings[readIndex];
  readings[readIndex] = RPM;
  total = total + readings[readIndex];
  readIndex = readIndex + 1;

  if (readIndex >= numReadings) {
    readIndex = 0;
  }
  average = total / numReadings;

  Key = myKeypad.getKey();
  if (Key == '*') {
    lcd.clear();
    delay(1000);
    return;
  }
  if (Key == "#") {
    lcd.clear();
    x = 0;
    kp, nilai_kp = 0;
    ki, nilai_ki = 0;
    kd, nilai_kd = 0;
    sp, nilai_sp = 0;
    pid = 0;
    delay(100);
    return;
  }
  ErrorX = Error;
  mulai();
}

void Pulse_Event() {
  PeriodBetweenPulses = micros() - LastTimeWeMeasured;
  LastTimeWeMeasured = micros();
  if (PulseCounter >= AmountOfReadings)  {
    PeriodAverage = PeriodSum / AmountOfReadings;
    PulseCounter = 1;
    PeriodSum = PeriodBetweenPulses;

    int RemapedAmountOfReadings = map(PeriodBetweenPulses, 40000, 5000, 1, 10);
    RemapedAmountOfReadings = constrain(RemapedAmountOfReadings, 1, 10);
    AmountOfReadings = RemapedAmountOfReadings;
  } else {
    PulseCounter++;
    PeriodSum = PeriodSum + PeriodBetweenPulses;
  }
}

void loop() {

  //function key
  Key = myKeypad.getKey();

  if (x == 0) {
    lcd.setCursor(0, 0);
    lcd.print("     Welcome    ");
  }
  if (x == 1) {

    lcd.setCursor(0, 0);
    lcd.print("1.SET PID       ");
  }

  if (x == 2) {

    lcd.setCursor(0, 0);
    lcd.print("2.CEK PID       ");
  }

  if (x == 3) {

    lcd.setCursor(0, 0);
    lcd.print("3.MULAI PID     ");
  }

  switch (Key) {
    case '0' ... '9':
      break;

    case '#':
      break;

    case '*':
      break;

    case 'A':
      x--;
      break;

    case 'B':
      x++;
      break;

    case 'C':
      break;

    case 'D':
      if (x == 0) {
        lcd.clear();
      }
      if (x == 1) {
        lcd.clear();
        set_kp();
        set_ki();
        set_kd();
        set_sp();
      }
      if (x == 2) {
        lcd.clear();
        cekpid();
      }
      if (x == 3) {
        lcd.clear();
        mulai();
      }
      break;
  }

  if (x > 3) {
    x = 0;
  }

  if (x < 0) {
    x = 3;
  }
}
