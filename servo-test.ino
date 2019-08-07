// ------------------------------------------------------
//I2C
#include <Wire.h>
//serwa połączone do PCA9685
#include <Adafruit_PWMServoDriver.h>
//serwa połączone bezpośrednio do Arduino
#include <Servo.h>
//BT połączone do pinów 5 i 4
#include <SoftwareSerial.h>
// ------------------------------------------------------

// ------------------------------------------------------
//adresy serw
#define SERVO_RIGHT_REAR0NUM   2
#define SERVO_RIGHT_REAR1NUM   1
#define SERVO_RIGHT_REAR2NUM   0
#define SERVO_RIGHT_MID0NUM    5
#define SERVO_RIGHT_MID1NUM    4
#define SERVO_RIGHT_MID2NUM    3
#define SERVO_RIGHT_FRONT0NUM  11 //numer cyfrowego pinu - serwo bezpośrednio połączone do Arduino
#define SERVO_RIGHT_FRONT1NUM  6
#define SERVO_RIGHT_FRONT2NUM  7
#define SERVO_LEFT_REAR0NUM    13
#define SERVO_LEFT_REAR1NUM    14
#define SERVO_LEFT_REAR2NUM    15
#define SERVO_LEFT_MID0NUM     10
#define SERVO_LEFT_MID1NUM     11
#define SERVO_LEFT_MID2NUM     12
#define SERVO_LEFT_FRONT0NUM   3  //numer cyfrowego pinu - serwo bezpośrednio połączone do Arduino
#define SERVO_LEFT_FRONT1NUM   9
#define SERVO_LEFT_FRONT2NUM   8
//minimalne i maksymalne długości impulsu
#define SERVOMIN  150 // minimalna długość impulsu (z 4096)
#define SERVOMAX  600 // minimalna długość impulsu (z 4096)
#define SERVOMID  375 //(SERVOMAX+SERVOMIN)/2 // wartość wyśrodkowana
// 450pul. - 180st. -> Xpul. - 62st. -> X = 155pul. (62st.) // a 60st. to 150pul.
#define SERVOMIN_0 /*220*//*225*//*300*//*325*/340 // wg pomiarów - 62st. od pozycji środkowej (150+225-155 = 220) // zakres 28st. -> 90 - 14 = 76st.
#define SERVOMAX_0 /*530*//*525*//*450*//*425*/410 // wg pomiarów - 62st. od pozycji środkowej (600-225+155 = 530) // zakres 28st. -> 90 + 14 = 104st.
#define SERVOMIN_1 /*215*//*245*/230
#define SERVOMAX_1 /*535*//*505*/520
#define SERVOMIN_2 /*205*/275
#define SERVOMAX_2 /*545*/475
// ------------------------------------------------------
//stany graniczne napięć baterii
#define BATTERY_CRITICAL 6.68
#define BATTERY_MEDIUM 7.11
// ------------------------------------------------------
//domyślna wartość ograniczenia (dla funkcji limitVal(value, limit))
#define DEFAULT_VALUE_LIMIT 73
// ------------------------------------------------------
//pozycja kolan i bioder, przy której robot jest w pozycji stojącej (nie może być większe niż 49)
#define STANDING_POSITION 38
// ------------------------------------------------------
//buzzer
#define BUZZER_PIN 12
// ------------------------------------------------------

enum RobotState   // stany robota
{
    Initialising = -1,// robot w stanie inicjalizacji
    Standing = 0,     // bezruch - OK
    MovingFront,      // robot jest w ruchu do przodu
    MovingBack,       // robot jest w ruchu do tyłu
    MovingLeft,       // robot jest w ruchu w lewo
    MovingRight,      // robot jest w ruchu w prawo
    TurningLeft,      // robot skręca w lewo
    TurningRight,     // robot skręca w prawo
//    Calibrating,      // robot w trakcie kalibracji
    Inactive          // robot nieaktywny - powinien być w pozycji początkowej
};

enum BatteryState // stany baterii
{
    BatteryOK = 0,    // poziom baterii OK
    BatteryLow        // niski poziom baterii
};

enum RobotCommand // rozkazy dla robota
{
    Stand = 0,        // robot stoi
    MoveFront,        // robot idzie do przodu
    MoveBack,         // robot idzie do tyłu
    MoveLeft,         // robot idzie w lewo
    MoveRight,        // robot idzie w prawo
    TurnLeft,         // robot skręca w lewo
    TurnRight,        // robot skręca w prawo
//    Calibrate,        // robot środkuje pozycje wszystkich serw (Należy trzymać robota w powietrzu i dać swobodę kończynom)
    GoToInitialPos    // robot wraca do pozycji początkowej
};

bool isActive = true;             // flaga mówiąca o aktywności programu głównego robota (po zmianie stanu robota na Inactive i obsłużeniu go, flaga ta zmienia stan)

auto servos = Adafruit_PWMServoDriver();
Servo servoLeft;
Servo servoRight;
SoftwareSerial btSerial(5,4);      // Bluetooth(rx, tx)

RobotState state = Initialising;
BatteryState battState = BatteryOK;
RobotCommand lastCommand = Stand;

unsigned long timeNow = 0;
unsigned long timeSaved = 0;

String receivedData;          // zmienna na dane odbierane przez Bluetooth

unsigned long buzzerDuration = 0;

// Prawe przednie serwo przy ciele robota
struct RightFrontBodyServo
{
  // Ustawia pozycję serwa (value - od 0 do 100, % odchylenia;)
  static void setPosition(int8_t value) {
    servoRight.write(map(value, 0, 100, /*120, 60*//*110, 70*/104, 76));
  }
};

// Prawe przednie serwo - "biodro" robota
struct RightFrontHipServo
{
  // Ustawia pozycję serwa (value - od 0 do 100, % odchylenia;)
  static void setPosition(int8_t value) {
    servos.setPWM(SERVO_RIGHT_FRONT1NUM, 0, map(value, 0, 100, SERVOMIN_1, SERVOMAX_1));
  }
};

// Prawe przednie serwo - "kolano" robota
struct RightFrontKneeServo
{
  // Ustawia pozycję serwa (value - od 0 do 100, % odchylenia;)
  static void setPosition(int8_t value) {
    servos.setPWM(SERVO_RIGHT_FRONT2NUM, 0, map(value, 0, 100, SERVOMAX_2, SERVOMIN_2));
  }
};


// Prawe środkowe serwo przy ciele robota
struct RightMiddleBodyServo
{
  // Ustawia pozycję serwa (value - od 0 do 100, % odchylenia;)
  static void setPosition(int8_t value) {
    servos.setPWM(SERVO_RIGHT_MID0NUM, 0, map(value, 0, 100, SERVOMAX_0, SERVOMIN_0));
  }
};

// Prawe środkowe serwo - "biodro" robota
struct RightMiddleHipServo
{
  // Ustawia pozycję serwa (value - od 0 do 100, % odchylenia;)
  static void setPosition(int8_t value) {
    servos.setPWM(SERVO_RIGHT_MID1NUM, 0, map(value, 0, 100, SERVOMAX_1, SERVOMIN_1));
  }
};

// Prawe środkowe serwo - "kolano" robota
struct RightMiddleKneeServo
{
  // Ustawia pozycję serwa (value - od 0 do 100, % odchylenia;)
  static void setPosition(int8_t value) {
    servos.setPWM(SERVO_RIGHT_MID2NUM, 0, map(value, 0, 100, SERVOMIN_2, SERVOMAX_2));
  }
};


// Prawe tylne serwo przy ciele robota
struct RightRearBodyServo
{
  // Ustawia pozycję serwa (value - od 0 do 100, % odchylenia;)
  static void setPosition(int8_t value) {
    servos.setPWM(SERVO_RIGHT_REAR0NUM, 0, map(value, 0, 100, SERVOMAX_0, SERVOMIN_0));
  }
};

// Prawe tylne serwo - "biodro" robota
struct RightRearHipServo
{
  // Ustawia pozycję serwa (value - od 0 do 100, % odchylenia;)
  static void setPosition(int8_t value) {
    servos.setPWM(SERVO_RIGHT_REAR1NUM, 0, map(value, 0, 100, SERVOMAX_1, SERVOMIN_1));
  }
};

// Prawe tylne serwo - "kolano" robota
struct RightRearKneeServo
{
  // Ustawia pozycję serwa (value - od 0 do 100, % odchylenia;)
  static void setPosition(int8_t value) {
    servos.setPWM(SERVO_RIGHT_REAR2NUM, 0, map(value, 0, 100, SERVOMIN_2, SERVOMAX_2));
  }
};



// Lewe przednie serwo przy ciele robota
struct LeftFrontBodyServo
{
  // Ustawia pozycję serwa (value - od 0 do 100, % odchylenia;)
  static void setPosition(int8_t value) {
    servoLeft.write(map(value, 0, 100, /*60, 120*//*70, 110*/76, 104));
  }
};

// Lewe przednie serwo - "biodro" robota
struct LeftFrontHipServo
{
  // Ustawia pozycję serwa (value - od 0 do 100, % odchylenia;)
  static void setPosition(int8_t value) {
    servos.setPWM(SERVO_LEFT_FRONT1NUM, 0, map(value, 0, 100, SERVOMAX_1, SERVOMIN_1));
  }
};

// Lewe przednie serwo - "kolano" robota
struct LeftFrontKneeServo
{
  // Ustawia pozycję serwa (value - od 0 do 100, % odchylenia;)
  static void setPosition(int8_t value) {
    servos.setPWM(SERVO_LEFT_FRONT2NUM, 0, map(value, 0, 100, SERVOMIN_2, SERVOMAX_2));
  }
};


// Lewe środkowe serwo przy ciele robota
struct LeftMiddleBodyServo
{
  // Ustawia pozycję serwa (value - od 0 do 100, % odchylenia;)
  static void setPosition(int8_t value) {
    servos.setPWM(SERVO_LEFT_MID0NUM, 0, map(value, 0, 100, SERVOMIN_0, SERVOMAX_0));
  }
};

// Lewe środkowe serwo - "biodro" robota
struct LeftMiddleHipServo
{
  // Ustawia pozycję serwa (value - od 0 do 100, % odchylenia;)
  static void setPosition(int8_t value) {
    servos.setPWM(SERVO_LEFT_MID1NUM, 0, map(value, 0, 100, SERVOMIN_1, SERVOMAX_1));
  }
};

// Lewe środkowe serwo - "kolano" robota
struct LeftMiddleKneeServo
{
  // Ustawia pozycję serwa (value - od 0 do 100, % odchylenia;)
  static void setPosition(int8_t value) {
    servos.setPWM(SERVO_LEFT_MID2NUM, 0, map(value, 0, 100, SERVOMAX_2, SERVOMIN_2));
  }
};


// Lewe tylne serwo przy ciele robota
struct LeftRearBodyServo
{
  // Ustawia pozycję serwa (value - od 0 do 100, % odchylenia;)
  static void setPosition(int8_t value) {
    servos.setPWM(SERVO_LEFT_REAR0NUM, 0, map(value, 0, 100, SERVOMIN_0, SERVOMAX_0));
  }
};

// Lewe tylne serwo - "biodro" robota
struct LeftRearHipServo
{
  // Ustawia pozycję serwa (value - od 0 do 100, % odchylenia;)
  static void setPosition(int8_t value) {
    servos.setPWM(SERVO_LEFT_REAR1NUM, 0, map(value, 0, 100, SERVOMIN_1, SERVOMAX_1));
  }
};

// Lewe tylne serwo - "kolano" robota
struct LeftRearKneeServo
{
  // Ustawia pozycję serwa (value - od 0 do 100, % odchylenia;)
  static void setPosition(int8_t value) {
    servos.setPWM(SERVO_LEFT_REAR2NUM, 0, map(value, 0, 100, SERVOMAX_2, SERVOMIN_2));
  }
};


void setup() {
  Serial.begin(9600);                     // inicjalizacja portu szeregowego
  Serial.println("Hello, RoboSpider here! I'm initialising now...");
  pinMode(BUZZER_PIN, OUTPUT);            // buzzer
  if (!isBatteryVoltageOkay()) {                // jeśli napięcie baterii nieprawidłowe
    return;                                                     // przerywam
  }
  servoInit();                          // inicjalizacja serw
  btSerial.begin(9600);                                         // inicjalizacja obsługi BT
  buzzOnce();                         // pojedynczy sygnał mówiący o prawidłowej inicjalizacji
  delay(500);                         // chwila "odpoczynku"..
}

// Zwraca true, jeśli stan baterii jest w porządku
bool isBatteryVoltageOkay() {
  if (readBatteryVoltage() < BATTERY_CRITICAL) {                // jeśli bardzo niskie napięcie baterii
    Serial.println("Error. Battery voltage level too low...");  // wyświetlam komunikat
    battState = BatteryLow;                   // ustawiam stan baterii
    state = Inactive;                     // oraz stan robota
    buzzTwice();
    return false;
  }
  return true;
}

// Odczytuje przybliżoną wartość napięcia baterii [V]
double readBatteryVoltage() {
  return static_cast<double>(analogRead(A6)) * 8.4/1024;
}

// Uruchamia buzzer na jedno krótkie piknięcie
void buzzOnce() {
  buzzerOn();                        // włączam buzzer
  delay(80);
  buzzerOff();
}

// Uruchamia buzzer na dwa krótkie piknięcia
void buzzTwice() {
  buzzOnce();
  delay(40);
  buzzOnce();
}

// Załącza buzzer
void buzzerOn() {
  digitalWrite(BUZZER_PIN, HIGH);
}

// Wyłącza buzzer
void buzzerOff() {
  digitalWrite(BUZZER_PIN, LOW);
  buzzerDuration = 0;
}

// Sprawdza stan buzzer
bool isBuzzerTurnedOn() {
  if (digitalRead(BUZZER_PIN) == HIGH) {
    return true;
  }
  return false;
}

// Inicjalizacja serw
void servoInit() {
  servos.begin();                                               // inicjalizacja sterownika PWM
  servos.setPWMFreq(60);                                        // analogowe serwa działają na około 60Hz
// PRZYDAŁOBY SIĘ JESZCZE ZROBIĆ COŚ Z DZIWNYM RUCHEM SERW NA POCZĄTKU
  delay(320);
  servoLeft.attach(SERVO_LEFT_FRONT0NUM);                       // inicjalizacja lewego serwa, podłączonego bezpośrednio do arduino
  delay(20);
  servoRight.attach(SERVO_RIGHT_FRONT0NUM);                     // inicjalizacja prawego serwa, podłączonego bezpośrednio do arduino
  delay(20);
  leftSideFrontBack(50);
  rightSideFrontBack(50);
  leftSideUpDown(100, 0b111);
  rightSideUpDown(100, 0b111);
  // pozycja początkowa: serwa najbliżej ciała robota wyśrodkowane, serwa "biodra" maksymalnie uniesione, serwa "kolana" maksymalnie opuszczone - kończyny "złożone"
}

// Wykonuje w nieskończonej pętli
void loop() {
  if (!isActive) {
    return;
  }
  readSerialData();
  readBluetoothData();
  checkIfBuzzerNeedsToGoOff();
  checkBatteryVoltageEveryTenSeconds();
  setLastCommandValue();
  robotMovement_CheckState();
}

// Odczytuje dane z portu szeregowego (np. wiadomość "1example" zostanie przetworzona na wartość 1)
void readSerialData() {
  if (Serial.available()) {
    String data;                        // tworzę pustą zmienną na dane
    data += Serial.readString().charAt(0);            // odczytuję serię danych z portu szeregowego, a pierwszy znak dopisuję do zmiennej
    receivedData = "";                      // zeruję globalną zmienną na dane
    receivedData += static_cast<char>(data.toInt());            // zamieniam dane zawierające jeden znak na integer, rzutuję na char i dopisuję do zmiennej globalnej
    Serial.println("I got some data from serial port!");
  }
}

// Odczytuje dane z modułu Bluetooth
void readBluetoothData() {
  if (btSerial.available()) {
    receivedData = btSerial.readString();   // zapis serii odebranych bajtów do zmiennej
  }
}

// Sprawdza czy buzzer nie powinien zostać wyłączony
void checkIfBuzzerNeedsToGoOff() {
  timeNow = millis();
  if (buzzerDuration > 0) {
    if (timeNow - timeSaved >= buzzerDuration) {
      buzzerOff();
    }
  }
}

// Sprawdza stan baterii co około 10 sekund
void checkBatteryVoltageEveryTenSeconds() {
  //timeNow = millis();
  if (timeNow - timeSaved >= 10000UL) {         // jeśli minęło więcej niż 10 sekund -> sprawdzenie napięcia baterii
    timeSaved = timeNow;
    checkBatteryState();
  }
}

// Sprawdza stan baterii
void checkBatteryState() {
  double batteryVoltage = readBatteryVoltage();
//  Serial.println(batteryVoltage);
  if (batteryVoltage < BATTERY_CRITICAL) {
    battState = BatteryLow;
    Serial.println("Warning. Low battery voltage level.");
    buzzerOn();
    buzzerDuration = 500;
  }
  else if (batteryVoltage < BATTERY_MEDIUM) {
    Serial.println("Medium battery voltage level. It is advised to turn off the robot and start recharging.");
    buzzerOn();
    buzzerDuration = 200;
  }
}

// Przypisuje ostatnio odebraną komendę do zmiennej
void setLastCommandValue() {
  if ((state != Inactive) && (battState != BatteryLow)) {
    if (receivedData.length()) {                    // jeśli jakiekolwiek dane odebrane
      lastCommand = static_cast<RobotCommand>(receivedData.charAt(0));  // zrzutowanie pierwszego bajtu ostatnio odebranej serii na enum komendy robota
    }
  }
  else {
    if (lastCommand != GoToInitialPos) {
      lastCommand = GoToInitialPos;
    }
  }
}

// Sprawdza bieżący stan robota i podejmuje odpowiednie działanie zależnie od jego wartości
void robotMovement_CheckState() {
  switch (state) {
  case Standing:
    stateStanding();
    break;
  case MovingFront:
    stateMovingFront();
    break;
  case MovingBack:
    stateMovingBack();
    break;
  case MovingLeft:
    stateMovingLeft();
    break;
  case MovingRight:
    stateMovingRight();
    break;
  case TurningLeft:
    stateTurningLeft();
    break;
  case TurningRight:
    stateTurningRight();
    break;
//  case Calibrating:
//    stateCalibrating();
//    break;
  case Inactive:
    stateInactive();
    break;
  case Initialising:
    stateInitialising();
    break;
  }
}

// Funkcja bazowa, sprawdzająca ostatnio odebraną komendę i zależnie od niej podejmująca odpowiednie działanie
void stateStanding() {
  switch(lastCommand) {
  case Stand:
    stillStand();
    break;
  case MoveFront:
    standToFront();
    state = MovingFront;
    break;
  case MoveBack:
    standToBack();
    state = MovingBack;
    break;
  case MoveLeft:
    standToLeft();
    state = MovingLeft;
    break;
  case MoveRight:
    standToRight();
    state = MovingRight;
    break;
  case TurnLeft:
    standToTurnLeft();
    state = TurningLeft;
    break;
  case TurnRight:
    standToTurnRight();
    state = TurningRight;
    break;
//  case Calibrate:
//    standToCalibrate();
//    state = Calibrating;
//    break;
  case GoToInitialPos:
    standToInitialPos();
    state = Inactive;
    break;
  }
}

// Sprawdza ostatnio odebraną komendę i zależnie od niej podejmuje odpowiednie działanie
void stateMovingFront() {
  switch(lastCommand) {
  case MoveFront:
    stillFront();
    break;
  default:
    frontToStand();
    state = Standing;
    break;
  }
}

// Sprawdza ostatnio odebraną komendę i zależnie od niej podejmuje odpowiednie działanie
void stateMovingBack() {
  switch(lastCommand) {
  case MoveBack:
    stillBack();
    break;
  default:
    backToStand();
    state = Standing;
    break;
  }
}

// Sprawdza ostatnio odebraną komendę i zależnie od niej podejmuje odpowiednie działanie
void stateMovingLeft() {
  switch(lastCommand) {
  case MoveLeft:
    stillLeft();
    break;
  default:
    leftToStand();
    state = Standing;
    break;
  }
}

// Sprawdza ostatnio odebraną komendę i zależnie od niej podejmuje odpowiednie działanie
void stateMovingRight() {
  switch(lastCommand) {
  case MoveRight:
    stillRight();
    break;
  default:
    rightToStand();
    state = Standing;
    break;
  }
}

// Sprawdza ostatnio odebraną komendę i zależnie od niej podejmuje odpowiednie działanie
void stateTurningLeft() {
  switch(lastCommand) {
  case TurnLeft:
    stillTurningLeft();
    break;
  default:
    turningLeftToStand();
    state = Standing;
    break;
  }
}

// Sprawdza ostatnio odebraną komendę i zależnie od niej podejmuje odpowiednie działanie
void stateTurningRight() {
  switch(lastCommand) {
  case TurnRight:
    stillTurningRight();
    break;
  default:
    turningRightToStand();
    state = Standing;
    break;
  }
}

// Sprawdza ostatnio odebraną komendę i zależnie od niej podejmuje odpowiednie działanie
//void stateCalibrating() {
//  switch (lastCommand) {
//  case Calibrating:
//    stillCalibrating();
//    break;
//  default:
//    calibratingToStand();
//    state = Standing;
//    break;
//  }
//}

// Kończy obsługę modułu Bluetooth, serw oraz ustawia flagę przetwarzania głównej pętli programu na false
void stateInactive() {
  btSerial.end();
//  Serial.end();
  isActive = false;
  servoLeft.detach();
  servoRight.detach();
  Wire.end();
  if (isBuzzerTurnedOn()) {
    buzzerOff();
  }
}

// Każe robotowi stanąć na kończynach, ustawia jego stan oraz wyświetla komunikat o zakończeniu inicjalizacji
void stateInitialising() {
  initialPosToStand();
  state = Standing;
  Serial.println("Done!");
}

// RUCHY ROBOTA:
void stillStand() {
  leftSideFrontBack(50);
  rightSideFrontBack(50);
  leftSideUpDown(STANDING_POSITION, 0b111);
  rightSideUpDown(STANDING_POSITION, 0b111);
}
// NA CHWILE WPROWADZONO DELAY(2)!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! pamietac, aby usunac
void standToFront() {
  for (int8_t i = 50; i < 75; ++i)  {
    leftSideFrontBack(i);
    rightSideFrontBack(i);
  int8_t j = map(i, 50, 100, STANDING_POSITION, 100);
    leftSideUpDown(limitVal(j), 0b101);
    rightSideUpDown(limitVal(j), 0b010);
    //delay(1);
  }
  //delay(1);
  for (int8_t i = 75; i <= 100; ++i) {
    leftSideFrontBack(i);
    rightSideFrontBack(i);
  int8_t j = map(i, 50, 100, 100, STANDING_POSITION);
    leftSideUpDown(limitVal(j), 0b101);
    rightSideUpDown(limitVal(j), 0b010);
    //delay(1);
  }
  for (int8_t i = 100; i > 50; --i) {
    leftSideFrontBack(i);
    rightSideFrontBack(i);
    int8_t j = map(i, 100, 50, STANDING_POSITION, 100);
    leftSideUpDown(limitVal(j), 0b010);
    rightSideUpDown(limitVal(j), 0b101);
    //delay(1);
  }
  //delay(1);
  for (int8_t i = 50; i >= 0; --i) {
    leftSideFrontBack(i);
    rightSideFrontBack(i);
    int8_t j = map(i, 50, 0, 100, STANDING_POSITION);
    leftSideUpDown(limitVal(j), 0b010);
    rightSideUpDown(limitVal(j), 0b101);
    //delay(1);
  }
}

void standToBack() {
  for (int8_t i = 50; i > 25; --i) {
    leftSideFrontBack(i);
    rightSideFrontBack(i);
    int8_t j = map(i, 50, 0, STANDING_POSITION, 100);
    leftSideUpDown(limitVal(j), 0b101);
    rightSideUpDown(limitVal(j), 0b010);
    //delay(1);
  }
  //delay(1);
  for (int8_t i = 25; i >= 0; --i) {
    leftSideFrontBack(i);
    rightSideFrontBack(i);
    int8_t j = map(i, 50, 0, 100, STANDING_POSITION);
    leftSideUpDown(limitVal(j), 0b101);
    rightSideUpDown(limitVal(j), 0b010);
    //delay(1);
  }
  for (int8_t i = 0; i < 50; ++i) {
    leftSideFrontBack(i);
    rightSideFrontBack(i);
    int8_t j = map(i, 0, 50, STANDING_POSITION, 100);
    leftSideUpDown(limitVal(j), 0b010);
    rightSideUpDown(limitVal(j), 0b101);
    //delay(1);
  }
  //delay(1);
  for (int8_t i = 50; i <= 100; ++i) {
    leftSideFrontBack(i);
    rightSideFrontBack(i);
    int8_t j = map(i, 50, 100, 100, STANDING_POSITION);
    leftSideUpDown(limitVal(j), 0b010);
    rightSideUpDown(limitVal(j), 0b101);
    //delay(1);
  }
}

void standToLeft() {
  //
}

void standToRight() {
  //
}

void standToTurnLeft() {
  for (int8_t i = 50; i < 75; ++i) {
    leftSideFrontBack(100 - i);
    rightSideFrontBack(i);
    int8_t j = map(i, 50, 100, STANDING_POSITION, 100);
    leftSideUpDown(limitVal(j), 0b101);
    rightSideUpDown(limitVal(j), 0b010);
    //delay(1);
  }
  //delay(1);
  for (int8_t i = 75; i <= 100; ++i) {
    leftSideFrontBack(100 - i);
    rightSideFrontBack(i);
    int8_t j = map(i, 50, 100, 100, STANDING_POSITION);
    leftSideUpDown(limitVal(j), 0b101);
    rightSideUpDown(limitVal(j), 0b010);
    //delay(1);
  }
  for (int8_t i = 100; i > 50; --i) {
    leftSideFrontBack(100 - i);
    rightSideFrontBack(i);
    int8_t j = map(i, 100, 50, STANDING_POSITION, 100);
    leftSideUpDown(limitVal(j), 0b010);
    rightSideUpDown(limitVal(j), 0b101);
    //delay(1);
  }
  //delay(1);
  for (int8_t i = 50; i >= 0; --i) {
    leftSideFrontBack(100 - i);
    rightSideFrontBack(i);
    int8_t j = map(i, 50, 0, 100, STANDING_POSITION);
    leftSideUpDown(limitVal(j), 0b010);
    rightSideUpDown(limitVal(j), 0b101);
    //delay(1);
  }
}

void standToTurnRight() {
  for (int8_t i = 50; i < 75; ++i) {
    leftSideFrontBack(i);
    rightSideFrontBack(100 - i);
    int8_t j = map(i, 50, 100, STANDING_POSITION, 100);
    leftSideUpDown(limitVal(j), 0b101);
    rightSideUpDown(limitVal(j), 0b010);
    //delay(1);
  }
  //delay(1);
  for (int8_t i = 75; i <= 100; ++i) {
    leftSideFrontBack(i);
    rightSideFrontBack(100 - i);
    int8_t j = map(i, 50, 100, 100, STANDING_POSITION);
    leftSideUpDown(limitVal(j), 0b101);
    rightSideUpDown(limitVal(j), 0b010);
    //delay(1);
  }
  for (int8_t i = 100; i > 50; --i) {
    leftSideFrontBack(i);
    rightSideFrontBack(100 - i);
    int8_t j = map(i, 100, 50, STANDING_POSITION, 100);
    leftSideUpDown(limitVal(j), 0b010);
    rightSideUpDown(limitVal(j), 0b101);
    //delay(1);
  }
  //delay(1);
  for (int8_t i = 50; i >= 0; --i) {
    leftSideFrontBack(i);
    rightSideFrontBack(100 - i);
    int8_t j = map(i, 50, 0, 100, STANDING_POSITION);
    leftSideUpDown(limitVal(j), 0b010);
    rightSideUpDown(limitVal(j), 0b101);
    //delay(1);
  }
}

//void standToCalibrate() {
//  servoRight.write(map(SERVOMID, SERVOMIN, SERVOMAX, 120, 60));
//  servoLeft.write(map(SERVOMID, SERVOMIN, SERVOMAX, 60, 120));
//  for (int8_t i = 0; i < 16; ++i) {
//    servos.setPWM(i, 0, SERVOMID);
//  }
//}

void standToInitialPos() {
  for (int8_t i = 50; i <= 100; ++i)
  {
    leftSideFrontBack(50);
    rightSideFrontBack(50);
  int8_t j = map(i, 50, 100, STANDING_POSITION, 100);
    leftSideUpDown(j, 0b111);
    rightSideUpDown(j, 0b111);
    //delay(1);
  }
}

void stillFront() {
  for (int8_t i = 0; i < 50; ++i) {
    leftSideFrontBack(i);
    rightSideFrontBack(i);
    //int8_t j = /*map(i, 0, 50, 50, 100)*/i + 50;
  int8_t j = map(i, 0, 50, STANDING_POSITION, 100);
    leftSideUpDown(limitVal(j), 0b101);
    rightSideUpDown(limitVal(j), 0b010);
    //delay(1);
  }
  //delay(1);
  for (int8_t i = 50; i <= 100; ++i) {
    leftSideFrontBack(i);
    rightSideFrontBack(i);
    //int8_t j = /*map(i, 50, 100, 100, 50)*/150 - i;
  int8_t j = map(i, 50, 100, 100, STANDING_POSITION);
    leftSideUpDown(limitVal(j), 0b101);
    rightSideUpDown(limitVal(j), 0b010);
    //delay(1);
  }
  for (int8_t i = 100; i > 50; --i) {
    leftSideFrontBack(i);
    rightSideFrontBack(i);
    //int8_t j = /*map(i, 100, 50, 50, 100)*/150 - i;
  int8_t j = map(i, 100, 50, STANDING_POSITION, 100);
    leftSideUpDown(limitVal(j), 0b010);
    rightSideUpDown(limitVal(j), 0b101);
    //delay(1);
  }
  //delay(1);
  for (int8_t i = 50; i >= 0; --i) {
    leftSideFrontBack(i);
    rightSideFrontBack(i);
    //int8_t j = /*map(i, 50, 0, 100, 50)*/i + 50;
  int8_t j = map(i, 50, 0, 100, STANDING_POSITION);
    leftSideUpDown(limitVal(j), 0b010);
    rightSideUpDown(limitVal(j), 0b101);
    //delay(1);
  }
}

void frontToStand() {
  for (int8_t i = 0; i < 25; ++i) {
    leftSideFrontBack(i);
    rightSideFrontBack(i);
  //int8_t j = /*map(i, 0, 25, 50, 75)*/i + 50;
  int8_t j = map(i, 0, 50, STANDING_POSITION, 100);
    leftSideUpDown(limitVal(j), 0b101);
    rightSideUpDown(limitVal(j), 0b010);
    //delay(1);
  }
  //delay(1);
  for (int8_t i = 25; i <= 50; ++i) {
    leftSideFrontBack(i);
    rightSideFrontBack(i);
    //int8_t j = /*map(i, 25, 50, 75, 50)*/100 - i;
  int8_t j = map(i, 0, 50, 100, STANDING_POSITION);
    leftSideUpDown(limitVal(j), 0b101);
    rightSideUpDown(limitVal(j), 0b010);
    //delay(1);
  }
}

void stillBack() {
  for (int8_t i = 100; i > 50; --i) {
    leftSideFrontBack(i);
    rightSideFrontBack(i);
    int8_t j = map(i, 100, 50, STANDING_POSITION, 100);
    leftSideUpDown(limitVal(j), 0b101);
    rightSideUpDown(limitVal(j), 0b010);
    //delay(1);
  }
  //delay(1);
  for (int8_t i = 50; i >= 0; --i) {
    leftSideFrontBack(i);
    rightSideFrontBack(i);
    int8_t j = map(i, 50, 0, 100, STANDING_POSITION);
    leftSideUpDown(limitVal(j), 0b101);
    rightSideUpDown(limitVal(j), 0b010);
    //delay(1);
  }
  for (int8_t i = 0; i < 50; ++i) {
    leftSideFrontBack(i);
    rightSideFrontBack(i);
    int8_t j = map(i, 0, 50, STANDING_POSITION, 100);
    leftSideUpDown(limitVal(j), 0b010);
    rightSideUpDown(limitVal(j), 0b101);
    //delay(1);
  }
  //delay(1);
  for (int8_t i = 50; i <= 100; ++i) {
    leftSideFrontBack(i);
    rightSideFrontBack(i);
    int8_t j = map(i, 50, 100, 100, STANDING_POSITION);
    leftSideUpDown(limitVal(j), 0b010);
    rightSideUpDown(limitVal(j), 0b101);
    //delay(1);
  }
}

void backToStand() {
  for (int8_t i = 100; i > 75; --i) {
    leftSideFrontBack(i);
    rightSideFrontBack(i);
    int8_t j = map(i, 100, 50, STANDING_POSITION, 100);
    leftSideUpDown(limitVal(j), 0b101);
    rightSideUpDown(limitVal(j), 0b010);
    //delay(1);
  }
  //delay(1);
  for (int8_t i = 75; i >= 50; --i) {
    leftSideFrontBack(i);
    rightSideFrontBack(i);
    int8_t j = map(i, 100, 50, 100, STANDING_POSITION);
    leftSideUpDown(limitVal(j), 0b101);
    rightSideUpDown(limitVal(j), 0b010);
    //delay(1);
  }
}

void stillLeft() {
  //
}

void leftToStand() {
  //
}

void stillRight() {
  //
}

void rightToStand() {
  //
}

void stillTurningLeft() {
  for (int8_t i = 0; i < 50; ++i) {
    leftSideFrontBack(100 - i);
    rightSideFrontBack(i);
    int8_t j = map(i, 0, 50, STANDING_POSITION, 100);
    leftSideUpDown(limitVal(j), 0b101);
    rightSideUpDown(limitVal(j), 0b010);
    //delay(1);
  }
  //delay(1);
  for (int8_t i = 50; i <= 100; ++i) {
    leftSideFrontBack(100 - i);
    rightSideFrontBack(i);
    int8_t j = map(i, 50, 100, 100, STANDING_POSITION);
    leftSideUpDown(limitVal(j), 0b101);
    rightSideUpDown(limitVal(j), 0b010);
    //delay(1);
  }
  for (int8_t i = 100; i > 50; --i) {
    leftSideFrontBack(100 - i);
    rightSideFrontBack(i);
    int8_t j = map(i, 100, 50, STANDING_POSITION, 100);
    leftSideUpDown(limitVal(j), 0b010);
    rightSideUpDown(limitVal(j), 0b101);
    //delay(1);
  }
  //delay(1);
  for (int8_t i = 50; i >= 0; --i) {
    leftSideFrontBack(100 - i);
    rightSideFrontBack(i);
    int8_t j = map(i, 50, 0, 100, STANDING_POSITION);
    leftSideUpDown(limitVal(j), 0b010);
    rightSideUpDown(limitVal(j), 0b101);
    //delay(1);
  }
}

void turningLeftToStand() {
  for (int8_t i = 0; i < 25; ++i) {
    leftSideFrontBack(100 - i);
    rightSideFrontBack(i);
    int8_t j = map(i, 0, 50, STANDING_POSITION, 100);
    leftSideUpDown(limitVal(j), 0b101);
    rightSideUpDown(limitVal(j), 0b010);
    //delay(1);
  }
  //delay(1);
  for (int8_t i = 25; i <= 50; ++i) {
    leftSideFrontBack(100 - i);
    rightSideFrontBack(i);
    int8_t j = map(i, 0, 50, 100, STANDING_POSITION);
    leftSideUpDown(limitVal(j), 0b101);
    rightSideUpDown(limitVal(j), 0b010);
    //delay(1);
  }
}

void stillTurningRight() {
  for (int8_t i = 0; i < 50; ++i) {
    leftSideFrontBack(i);
    rightSideFrontBack(100 - i);
    int8_t j = map(i, 0, 50, STANDING_POSITION, 100);
    leftSideUpDown(limitVal(j), 0b101);
    rightSideUpDown(limitVal(j), 0b010);
    //delay(1);
  }
  //delay(1);
  for (int8_t i = 50; i <= 100; ++i) {
    leftSideFrontBack(i);
    rightSideFrontBack(100 - i);
    int8_t j = map(i, 50, 100, 100, STANDING_POSITION);
    leftSideUpDown(limitVal(j), 0b101);
    rightSideUpDown(limitVal(j), 0b010);
    //delay(1);
  }
  for (int8_t i = 100; i > 50; --i) {
    leftSideFrontBack(i);
    rightSideFrontBack(100 - i);
    int8_t j = map(i, 100, 50, STANDING_POSITION, 100);
    leftSideUpDown(limitVal(j), 0b010);
    rightSideUpDown(limitVal(j), 0b101);
    //delay(1);
  }
  //delay(1);
  for (int8_t i = 50; i >= 0; --i) {
    leftSideFrontBack(i);
    rightSideFrontBack(100 - i);
    int8_t j = map(i, 50, 0, 100, STANDING_POSITION);
    leftSideUpDown(limitVal(j), 0b010);
    rightSideUpDown(limitVal(j), 0b101);
    //delay(1);
  }
}

void turningRightToStand() {
  for (int8_t i = 0; i < 25; ++i) {
    leftSideFrontBack(i);
    rightSideFrontBack(100 - i);
    int8_t j = map(i, 0, 50, STANDING_POSITION, 100);
    leftSideUpDown(limitVal(j), 0b101);
    rightSideUpDown(limitVal(j), 0b010);
    //delay(1);
  }
  //delay(1);
  for (int8_t i = 25; i <= 50; ++i) {
    leftSideFrontBack(i);
    rightSideFrontBack(100 - i);
    int8_t j = map(i, 0, 50, 100, STANDING_POSITION);
    leftSideUpDown(limitVal(j), 0b101);
    rightSideUpDown(limitVal(j), 0b010);
    //delay(1);
  }
}

//void stillCalibrating() {
//  servoRight.write(map(SERVOMID, SERVOMIN, SERVOMAX, 120, 60));
//  servoLeft.write(map(SERVOMID, SERVOMIN, SERVOMAX, 60, 120));
//  for (int8_t i = 0; i < 16; ++i) {
//    servos.setPWM(i, 0, SERVOMID);
//  }
//}

//void calibratingToStand() {
//  //
//}

void initialPosToStand() {
  for (int8_t i = 100; i >= 50; --i)
  {
    leftSideFrontBack(50);
    rightSideFrontBack(50);
    //leftSideUpDown(i, 0b111);
    //rightSideUpDown(i, 0b111);
    int8_t subtraction = 100 - i;
    RightFrontHipServo::setPosition(100);
    RightFrontKneeServo::setPosition(subtraction);
    RightMiddleHipServo::setPosition(100);
    RightMiddleKneeServo::setPosition(subtraction);
    RightRearHipServo::setPosition(100);
    RightRearKneeServo::setPosition(subtraction);
    LeftFrontHipServo::setPosition(100);
    LeftFrontKneeServo::setPosition(subtraction);
    LeftMiddleHipServo::setPosition(100);
    LeftMiddleKneeServo::setPosition(subtraction);
    LeftRearHipServo::setPosition(100);
    LeftRearKneeServo::setPosition(subtraction);
    delay(1);
  }
  for (int8_t i = 100; i >= 50; --i)
  {
    leftSideFrontBack(50);
    rightSideFrontBack(50);
    RightFrontHipServo::setPosition(i);
    RightFrontKneeServo::setPosition(50);
    RightMiddleHipServo::setPosition(i);
    RightMiddleKneeServo::setPosition(50);
    RightRearHipServo::setPosition(i);
    RightRearKneeServo::setPosition(50);
    LeftFrontHipServo::setPosition(i);
    LeftFrontKneeServo::setPosition(50);
    LeftMiddleHipServo::setPosition(i);
    LeftMiddleKneeServo::setPosition(50);
    LeftRearHipServo::setPosition(i);
    LeftRearKneeServo::setPosition(50);
    delay(1);
  }
  for (int8_t i = 49; i >= STANDING_POSITION; --i)
  {
    leftSideFrontBack(50);
    rightSideFrontBack(50);
    leftSideUpDown(i, 0b111);
    rightSideUpDown(i, 0b111);
    delay(1);
  }
}

// Zwraca wartość z przedziału od 0 do 100 przeskalowaną do wartości z przedziału od 0 do newMax
int8_t mMap(int8_t value, int8_t newMax) {
  return map(value, 0, 100, 0, newMax);
}

// Zwraca wartość parametru value nie większą niż domyślny parametr limitu (DEFAULT_VALUE_LIMIT)
int8_t limitVal(int8_t value) {
  return limitVal(value, DEFAULT_VALUE_LIMIT);
}
// Zwraca wartość parametru value nie większą niż podany parametr limit
int8_t limitVal(int8_t value, int8_t limit) {
  if (value >= limit) {
    return limit;
  }
  return value;
}

// Odnóża po prawej stronie dla value=100: skrajne na maksa do tyłu, środkowe na maksa do przodu
void rightSideFrontBack(int8_t value) {
  RightFrontBodyServo::setPosition(100 - value);
  RightMiddleBodyServo::setPosition(value);
  RightRearBodyServo::setPosition(100 - value);
}

// Odnóża po prawej stronie dla value=100: wybrane poprzez parametr select (bit0 - przód, bit1 - środek, bit2 - tył) na maksa do góry
void rightSideUpDown(int8_t value, int8_t select) {
  rightSideUpDown(value, select, STANDING_POSITION);
}
void rightSideUpDown(int8_t value, int8_t select, int8_t defStandVal) {
  int8_t subtraction = 100 - value;
  int8_t defSubtraction = 100 - defStandVal;
  RightFrontHipServo::setPosition((select & 0b100) ? value : defStandVal);
  RightFrontKneeServo::setPosition((select & 0b100) ? subtraction : defSubtraction);
  RightMiddleHipServo::setPosition((select & 0b010) ? value : defStandVal);
  RightMiddleKneeServo::setPosition((select & 0b010) ? subtraction : defSubtraction);
  RightRearHipServo::setPosition((select & 0b001) ? value : defStandVal);
  RightRearKneeServo::setPosition((select & 0b001) ? subtraction : defSubtraction);
}

// Odnóża po lewej stronie dla value=100: skrajne na maksa do przodu, środkowe na maksa do tyłu
void leftSideFrontBack(int8_t value) {
  LeftFrontBodyServo::setPosition(value);
  LeftMiddleBodyServo::setPosition(100 - value);
  LeftRearBodyServo::setPosition(value);
}

// Odnóża po lewej stronie dla value=100: wybrane poprzez parametr select (bit0 - przód, bit1 - środek, bit2 - tył) na maksa do góry
void leftSideUpDown(int8_t value, int8_t select) {
  leftSideUpDown(value, select, STANDING_POSITION);
}
void leftSideUpDown(int8_t value, int8_t select, int8_t defStandVal) {
  int8_t subtraction = 100 - value;
  int8_t defSubtraction = 100 - defStandVal;
  LeftFrontHipServo::setPosition((select & 0b100) ? value : defStandVal);
  LeftFrontKneeServo::setPosition((select & 0b100) ? subtraction : defSubtraction);
  LeftMiddleHipServo::setPosition((select & 0b010) ? value : defStandVal);
  LeftMiddleKneeServo::setPosition((select & 0b010) ? subtraction : defSubtraction);
  LeftRearHipServo::setPosition((select & 0b001) ? value : defStandVal);
  LeftRearKneeServo::setPosition((select & 0b001) ? subtraction : defSubtraction);
}
