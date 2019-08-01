// ------------------------------------------------------
//I2C
#include <Wire.h>
//serwa połąaczone do PCA9685
#include <Adafruit_PWMServoDriver.h>
//serwa połączone bezpośrednio do Arduino
#include <Servo.h>
//bt połączone do pinów 5 i 4
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
#define SERVOMIN_0 225 // wg pomiarów - 62st. od pozycji środkowej (150+225-155 = 220)
#define SERVOMAX_0 525 // wg pomiarów - 62st. od pozycji środkowej (600-225+155 = 530)
#define SERVOMIN_1 205
#define SERVOMAX_1 545
#define SERVOMIN_2 205
#define SERVOMAX_2 545
// ------------------------------------------------------
#define BATTERY_CRITICAL 6.08
#define BATTERY_MEDIUM 6.65
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
	Calibrating,      // robot w trakcie kalibracji
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
    Calibrate,        // robot środkuje pozycje wszystkich serw (Należy trzymać robota w powietrzu i dać swobodę kończynom)
    GoToInitialPos    // robot wraca do pozycji początkowej
};

auto servos = Adafruit_PWMServoDriver();
Servo servoLeft;
Servo servoRight;
SoftwareSerial btSerial(5,4);			// Bluetooth(rx, tx)

RobotState state = Initialising;
BatteryState battState = BatteryOK;
RobotCommand lastCommand = Stand;

uint16_t pulselen/* = SERVOMID*/;		// długość impulsu wysyłanego do sterownika PWM (12bit)
unsigned long timeNow = 0;
unsigned long timeSaved = 0;

String receivedData;					// zmienna na dane odbierane przez Bluetooth

// Prawe przednie serwo przy ciele robota
struct RightFrontBodyServo
{
	// Ustawia pozycję serwa (value - od 0 do 100, % odchylenia;)
	static void setPosition(uint8_t value/*, bool invert = false*/) {
		//if (!invert) {
		servoRight.write(map(value, 0, 100, 120, 60));
		/*}
		else {
			servoRight.write(map(value, 0, 100, 60, 120));
		}*/
	}
};

// Prawe przednie serwo - "biodro" robota
struct RightFrontHipServo
{
	// Ustawia pozycję serwa (value - od 0 do 100, % odchylenia;)
	static void setPosition(uint8_t value) {
		servos.setPWM(SERVO_RIGHT_FRONT1NUM, 0, map(value, 0, 100, SERVOMIN_1, SERVOMAX_1));
	}
};

// Prawe przednie serwo - "kolano" robota
struct RightFrontKneeServo
{
	// Ustawia pozycję serwa (value - od 0 do 100, % odchylenia;)
	static void setPosition(uint8_t value) {
		servos.setPWM(SERVO_RIGHT_FRONT2NUM, 0, map(value, 0, 100, SERVOMAX_2, SERVOMIN_2));
	}
};


// Prawe środkowe serwo przy ciele robota
struct RightMiddleBodyServo
{
	// Ustawia pozycję serwa (value - od 0 do 100, % odchylenia;)
	static void setPosition(uint8_t value) {
		servos.setPWM(SERVO_RIGHT_MID0NUM, 0, map(value, 0, 100, SERVOMAX_0, SERVOMIN_0));
	}
};

// Prawe środkowe serwo - "biodro" robota
struct RightMiddleHipServo
{
	// Ustawia pozycję serwa (value - od 0 do 100, % odchylenia;)
	static void setPosition(uint8_t value) {
		servos.setPWM(SERVO_RIGHT_MID1NUM, 0, map(value, 0, 100, SERVOMAX_1, SERVOMIN_1));
	}
};

// Prawe środkowe serwo - "kolano" robota
struct RightMiddleKneeServo
{
	// Ustawia pozycję serwa (value - od 0 do 100, % odchylenia;)
	static void setPosition(uint8_t value) {
		servos.setPWM(SERVO_RIGHT_MID2NUM, 0, map(value, 0, 100, SERVOMIN_2, SERVOMAX_2));
	}
};


// Prawe tylne serwo przy ciele robota
struct RightRearBodyServo
{
	// Ustawia pozycję serwa (value - od 0 do 100, % odchylenia;)
	static void setPosition(uint8_t value) {
		servos.setPWM(SERVO_RIGHT_REAR0NUM, 0, map(value, 0, 100, SERVOMAX_0, SERVOMIN_0));
	}
};

// Prawe tylne serwo - "biodro" robota
struct RightRearHipServo
{
	// Ustawia pozycję serwa (value - od 0 do 100, % odchylenia;)
	static void setPosition(uint8_t value) {
		servos.setPWM(SERVO_RIGHT_REAR1NUM, 0, map(value, 0, 100, SERVOMAX_1, SERVOMIN_1));
	}
};

// Prawe tylne serwo - "kolano" robota
struct RightRearKneeServo
{
	// Ustawia pozycję serwa (value - od 0 do 100, % odchylenia;)
	static void setPosition(uint8_t value) {
		servos.setPWM(SERVO_RIGHT_REAR2NUM, 0, map(value, 0, 100, SERVOMIN_2, SERVOMAX_2));
	}
};



// Lewe przednie serwo przy ciele robota
struct LeftFrontBodyServo
{
	// Ustawia pozycję serwa (value - od 0 do 100, % odchylenia;)
	static void setPosition(uint8_t value) {
		servoLeft.write(map(value, 0, 100, 60, 120));
	}
};

// Lewe przednie serwo - "biodro" robota
struct LeftFrontHipServo
{
	// Ustawia pozycję serwa (value - od 0 do 100, % odchylenia;)
	static void setPosition(uint8_t value) {
		servos.setPWM(SERVO_LEFT_FRONT1NUM, 0, map(value, 0, 100, SERVOMAX_1, SERVOMIN_1));
	}
};

// Lewe przednie serwo - "kolano" robota
struct LeftFrontKneeServo
{
	// Ustawia pozycję serwa (value - od 0 do 100, % odchylenia;)
	static void setPosition(uint8_t value) {
		servos.setPWM(SERVO_LEFT_FRONT2NUM, 0, map(value, 0, 100, SERVOMIN_2, SERVOMAX_2));
	}
};


// Lewe środkowe serwo przy ciele robota
struct LeftMiddleBodyServo
{
	// Ustawia pozycję serwa (value - od 0 do 100, % odchylenia;)
	static void setPosition(uint8_t value) {
		servos.setPWM(SERVO_LEFT_MID0NUM, 0, map(value, 0, 100, SERVOMIN_0, SERVOMAX_0));
	}
};

// Lewe środkowe serwo - "biodro" robota
struct LeftMiddleHipServo
{
	// Ustawia pozycję serwa (value - od 0 do 100, % odchylenia;)
	static void setPosition(uint8_t value) {
		servos.setPWM(SERVO_LEFT_MID1NUM, 0, map(value, 0, 100, SERVOMIN_1, SERVOMAX_1));
	}
};

// Lewe środkowe serwo - "kolano" robota
struct LeftMiddleKneeServo
{
	// Ustawia pozycję serwa (value - od 0 do 100, % odchylenia;)
	static void setPosition(uint8_t value) {
		servos.setPWM(SERVO_LEFT_MID2NUM, 0, map(value, 0, 100, SERVOMAX_2, SERVOMIN_2));
	}
};


// Lewe tylne serwo przy ciele robota
struct LeftRearBodyServo
{
	// Ustawia pozycję serwa (value - od 0 do 100, % odchylenia;)
	static void setPosition(uint8_t value) {
		servos.setPWM(SERVO_LEFT_REAR0NUM, 0, map(value, 0, 100, SERVOMIN_0, SERVOMAX_0));
	}
};

// Lewe tylne serwo - "biodro" robota
struct LeftRearHipServo
{
	// Ustawia pozycję serwa (value - od 0 do 100, % odchylenia;)
	static void setPosition(uint8_t value) {
		servos.setPWM(SERVO_LEFT_REAR1NUM, 0, map(value, 0, 100, SERVOMIN_1, SERVOMAX_1));
	}
};

// Lewe tylne serwo - "kolano" robota
struct LeftRearKneeServo
{
	// Ustawia pozycję serwa (value - od 0 do 100, % odchylenia;)
	static void setPosition(uint8_t value) {
		servos.setPWM(SERVO_LEFT_REAR2NUM, 0, map(value, 0, 100, SERVOMAX_2, SERVOMIN_2));
	}
};


void setup() {
  Serial.begin(9600);											// inicjalizacja portu szeregowego
  Serial.println("Hello, RoboSpider here! I'm initialising now...");
  if (!isBatteryVoltageOkay()) {								// jeśli napięcie baterii nieprawidłowe
    return;                                                     // przerywam
  }
  servoInit();													// inicjalizacja serw
  btSerial.begin(9600);                                         // inicjalizacja obsługi BT
  delay(500);													// chwila "odpoczynku"..
}

// Zwraca true, jeśli stan baterii jest w porządku
bool isBatteryVoltageOkay() {
  if (readBatteryVoltage() < BATTERY_CRITICAL) {                // jeśli bardzo niskie napięcie baterii
    Serial.println("Error. Battery voltage level too low...");	// wyświetlam komunikat
	battState = BatteryLow;										// ustawiam stan baterii
	state = Inactive;											// oraz stan robota
	return false;
  }
  return true;
}

// Odczytuje przybliżoną wartość napięcia baterii [V]
double readBatteryVoltage() {
  return static_cast<double>(analogRead(A6)) * 8.4/1024;
}

// Inicjalizacja serw
void servoInit() {
  servos.begin();                                               // inicjalizacja sterownika PWM
  servos.setPWMFreq(60);                                        // analogowe serwa działają na około 60Hz
  //  for (uint8_t servoNum = 0; servoNum < 16; servoNum++) {       // ustawienie pozycji początkowych
  //    if (servoNum == SERVO_RIGHT_REAR0NUM || servoNum == SERVO_RIGHT_MID0NUM
  //       || servoNum == SERVO_LEFT_REAR0NUM || servoNum == SERVO_LEFT_MID0NUM) {
  //      servos.setPWM(servoNum, 0, SERVOMID);
  //    }
  //    else {
  //      servos.setPWM(servoNum, 0, SERVOMIN);
  //    }
  //  }
  // PRZYDAŁOBY SIĘ JESZCZE ZROBIĆ COŚ Z DZIWNYM RUCHEM SERW NA POCZĄTKU
  /*servos.setPWM(SERVO_RIGHT_REAR0NUM, 0, SERVOMID);
  servos.setPWM(SERVO_RIGHT_MID0NUM, 0, SERVOMID);

  servos.setPWM(SERVO_LEFT_REAR0NUM, 0, SERVOMID);
  servos.setPWM(SERVO_LEFT_MID0NUM, 0, SERVOMID);

  servos.setPWM(SERVO_RIGHT_REAR1NUM, 0, SERVOMAX_1);
  servos.setPWM(SERVO_RIGHT_MID1NUM, 0, SERVOMAX_1);

  servos.setPWM(SERVO_LEFT_FRONT1NUM, 0, SERVOMAX_1);

  servos.setPWM(SERVO_RIGHT_REAR2NUM, 0, SERVOMIN_2);
  servos.setPWM(SERVO_RIGHT_MID2NUM, 0, SERVOMIN_2);
  
  servos.setPWM(SERVO_LEFT_FRONT2NUM, 0, SERVOMIN_2);

  servos.setPWM(SERVO_RIGHT_FRONT1NUM, 0, SERVOMIN_1);

  servos.setPWM(SERVO_LEFT_REAR1NUM, 0, SERVOMIN_1);
  servos.setPWM(SERVO_LEFT_MID1NUM, 0, SERVOMIN_1);

  servos.setPWM(SERVO_RIGHT_FRONT2NUM, 0, SERVOMAX_2);

  servos.setPWM(SERVO_LEFT_REAR2NUM, 0, SERVOMAX_2);
  servos.setPWM(SERVO_LEFT_MID2NUM, 0, SERVOMAX_2);*/

  servoLeft.attach(SERVO_LEFT_FRONT0NUM);                       // inicjalizacja lewego serwa, podłączonego bezpośrednio do arduino
  //servoLeft.write(90);                                          // ustawienie jego pozycji początkowej

  servoRight.attach(SERVO_RIGHT_FRONT0NUM);                     // inicjalizacja prawego serwa, podłączonego bezpośrednio do arduino
  //servoRight.write(90);                                         // ustawienie jego pozycji początkowej

  RightFrontBodyServo::setPosition(50);
  RightFrontHipServo::setPosition(100);
  RightFrontKneeServo::setPosition(0);
  RightMiddleBodyServo::setPosition(50);
  RightMiddleHipServo::setPosition(100);
  RightMiddleKneeServo::setPosition(0);
  RightRearBodyServo::setPosition(50);
  RightRearHipServo::setPosition(100);
  RightRearKneeServo::setPosition(0);
  LeftFrontBodyServo::setPosition(50);
  LeftFrontHipServo::setPosition(100);
  LeftFrontKneeServo::setPosition(0);
  LeftMiddleBodyServo::setPosition(50);
  LeftMiddleHipServo::setPosition(100);
  LeftMiddleKneeServo::setPosition(0);
  LeftRearBodyServo::setPosition(50);
  LeftRearHipServo::setPosition(100);
  LeftRearKneeServo::setPosition(0);

  // pozycja początkowa: serwa najbliżej ciała robota wyśrodkowane, a reszta na pozycji = minimum / maksimum zależnie od strony - kończyny "złożone"
}

// Wykonuje w nieskończonej pętli
void loop() {
  readBluetoothData();
  checkBatteryVoltageEveryTenSeconds();
  setLastCommandValue();
  robotMovement_CheckState();

/* //TEST
  for (pulselen = SERVOMID; pulselen < SERVOMAX; pulselen+=5) {
    servoLeft.write(map(pulselen, SERVOMIN, SERVOMAX, 60, 120));
    servoRight.write(map(pulselen, SERVOMIN, SERVOMAX, 120, 60));
    servos.setPWM(SERVO_RIGHT_REAR1NUM, 0, pulselen);
    servos.setPWM(SERVO_RIGHT_MID0NUM, 0, map(pulselen, SERVOMIN, SERVOMAX, SERVOMIN_LEVEL0, SERVOMAX_LEVEL0));
    servos.setPWM(SERVO_RIGHT_REAR0NUM, 0, map(pulselen, SERVOMIN, SERVOMAX, SERVOMIN_LEVEL0, SERVOMAX_LEVEL0));
    servos.setPWM(SERVO_LEFT_MID0NUM, 0, map(pulselen, SERVOMIN, SERVOMAX, SERVOMIN_LEVEL0, SERVOMAX_LEVEL0));
    servos.setPWM(SERVO_LEFT_REAR0NUM, 0, map(pulselen, SERVOMIN, SERVOMAX, SERVOMIN_LEVEL0, SERVOMAX_LEVEL0));
  }
  for (pulselen = SERVOMAX; pulselen > SERVOMID; pulselen-=5) {
    servoLeft.write(map(pulselen, SERVOMIN, SERVOMAX, 60, 120));
    servoRight.write(map(pulselen, SERVOMIN, SERVOMAX, 120, 60));
    servos.setPWM(SERVO_RIGHT_REAR1NUM, 0, pulselen);
    servos.setPWM(SERVO_RIGHT_MID0NUM, 0, map(pulselen, SERVOMIN, SERVOMAX, SERVOMIN_LEVEL0, SERVOMAX_LEVEL0));
    servos.setPWM(SERVO_RIGHT_REAR0NUM, 0, map(pulselen, SERVOMIN, SERVOMAX, SERVOMIN_LEVEL0, SERVOMAX_LEVEL0));
    servos.setPWM(SERVO_LEFT_MID0NUM, 0, map(pulselen, SERVOMIN, SERVOMAX, SERVOMIN_LEVEL0, SERVOMAX_LEVEL0));
    servos.setPWM(SERVO_LEFT_REAR0NUM, 0, map(pulselen, SERVOMIN, SERVOMAX, SERVOMIN_LEVEL0, SERVOMAX_LEVEL0));
  }
  for (pulselen = SERVOMID; pulselen > SERVOMIN; pulselen-=5) {
    servoLeft.write(map(pulselen, SERVOMIN, SERVOMAX, 60, 120));
    servoRight.write(map(pulselen, SERVOMIN, SERVOMAX, 120, 60));
    servos.setPWM(SERVO_RIGHT_REAR1NUM, 0, pulselen);
    servos.setPWM(SERVO_RIGHT_MID0NUM, 0, map(pulselen, SERVOMIN, SERVOMAX, SERVOMIN_LEVEL0, SERVOMAX_LEVEL0));
    servos.setPWM(SERVO_RIGHT_REAR0NUM, 0, map(pulselen, SERVOMIN, SERVOMAX, SERVOMIN_LEVEL0, SERVOMAX_LEVEL0));
    servos.setPWM(SERVO_LEFT_MID0NUM, 0, map(pulselen, SERVOMIN, SERVOMAX, SERVOMIN_LEVEL0, SERVOMAX_LEVEL0));
    servos.setPWM(SERVO_LEFT_REAR0NUM, 0, map(pulselen, SERVOMIN, SERVOMAX, SERVOMIN_LEVEL0, SERVOMAX_LEVEL0));
  }
  for (pulselen = SERVOMIN; pulselen < SERVOMID; pulselen+=5) {
    servoLeft.write(map(pulselen, SERVOMIN, SERVOMAX, 60, 120));
    servoRight.write(map(pulselen, SERVOMIN, SERVOMAX, 120, 60));
    servos.setPWM(SERVO_RIGHT_REAR1NUM, 0, pulselen);
    servos.setPWM(SERVO_RIGHT_MID0NUM, 0, map(pulselen, SERVOMIN, SERVOMAX, SERVOMIN_LEVEL0, SERVOMAX_LEVEL0));
    servos.setPWM(SERVO_RIGHT_REAR0NUM, 0, map(pulselen, SERVOMIN, SERVOMAX, SERVOMIN_LEVEL0, SERVOMAX_LEVEL0));
    servos.setPWM(SERVO_LEFT_MID0NUM, 0, map(pulselen, SERVOMIN, SERVOMAX, SERVOMIN_LEVEL0, SERVOMAX_LEVEL0));
    servos.setPWM(SERVO_LEFT_REAR0NUM, 0, map(pulselen, SERVOMIN, SERVOMAX, SERVOMIN_LEVEL0, SERVOMAX_LEVEL0));
  }
*/

/*
  for (pulselen = SERVOMID; pulselen < SERVOMAX; pulselen++) { // faza 1: od środka do końca
  //for (uint16_t pulselen = SERVOMID; pulselen < SERVOMAX; pulselen++) { // faza 1: od środka do końca
//  if (phase == 0 && pulselen >= SERVOMID && pulselen < SERVOMAX) {
    //servos.setPWM(SERVO_RIGHT_REAR1NUM, 0, map(pulselen, SERVOMIN, SERVOMAX, SERVOMIN, SERVOMAX));
    //servos.setPWM(SERVO_RIGHT_REAR1NUM, 0, pulselen++);
    phaseFirst();
  }
//  else if (phase == 0) {
//    phase = 1;
//    delay(350);
//  }

  delay(350);
  for (pulselen = SERVOMAX; pulselen > SERVOMID; pulselen--) { // faza 2: od końca do środka
  //for (uint16_t pulselen = SERVOMAX; pulselen > SERVOMID; pulselen--) { // faza 2: od końca do środka
//  if (phase == 1 && pulselen <= SERVOMAX && pulselen > SERVOMID) {
    //servos.setPWM(SERVO_RIGHT_REAR1NUM, 0, map(pulselen, SERVOMIN, SERVOMAX, SERVOMIN, SERVOMAX));
    //servos.setPWM(SERVO_RIGHT_REAR1NUM, 0, pulselen--);
    phaseSecond();
  }
//  else if (phase == 1) {
//    phase = 2;
//  }

  for (pulselen = SERVOMID; pulselen > SERVOMIN; pulselen--) { // faza 3: od środka do początku
  //for (uint16_t pulselen = SERVOMID; pulselen > SERVOMIN; pulselen--) { // faza 3: od środka do początku
//  if (phase == 2 && pulselen <= SERVOMID && pulselen > SERVOMIN) {
    //servos.setPWM(SERVO_RIGHT_REAR1NUM, 0, map(pulselen, SERVOMIN, SERVOMAX, SERVOMIN, SERVOMAX));
    //servos.setPWM(SERVO_RIGHT_REAR1NUM, 0, pulselen--);
    phaseThird();
  }
//  else if (phase == 2) {
//    phase = 3;
//    delay(350);
//  }

  delay(350);
  for (pulselen = SERVOMIN; pulselen < SERVOMID; pulselen++) { // faza 4: od początku do środka
  //for (uint16_t pulselen = SERVOMIN; pulselen < SERVOMID; pulselen++) { // faza 4: od początku do środka
//  if (phase == 3 && pulselen >= SERVOMIN && pulselen < SERVOMID) {
    //servos.setPWM(SERVO_RIGHT_REAR1NUM, 0, map(pulselen, SERVOMIN, SERVOMAX, SERVOMIN, SERVOMAX));
    //servos.setPWM(SERVO_RIGHT_REAR1NUM, 0, pulselen++);
    phaseFourth();
  }
//  else if (phase == 3) {
//    phase = 0;
//  }
*/
  
/*
  delay(500);
  Serial.println("pca w jedna strone");
  for (uint16_t pulselen = SERVOMIN; pulselen < SERVOMAX; pulselen++) {
    //servos.setPWM(SERVO_RIGHT_REAR1NUM, 0, pulselen);
    servos.setPWM(SERVO_RIGHT_REAR1NUM, 0, map(pulselen, SERVOMIN, SERVOMAX, SERVOMIN, SERVOMAX));
  }
  delay(500);
  Serial.println("pca w druga strone");
  //for (uint16_t pulselen = SERVOMAX; pulselen > SERVOMIN; pulselen--) {
  for (uint16_t pulselen = SERVOMIN; pulselen < SERVOMAX; pulselen++) {
    //servos.setPWM(SERVO_RIGHT_REAR1NUM, 0, pulselen);
    servos.setPWM(SERVO_RIGHT_REAR1NUM, 0, map(pulselen, SERVOMIN, SERVOMAX, SERVOMAX, SERVOMIN));
  }

  delay(500);

  Serial.println("lewa w jedna strone");
  for (uint16_t pulselen = SERVOMIN; pulselen < SERVOMAX; pulselen++) {
    servoLeft.write(map(pulselen, SERVOMIN, SERVOMAX, 60, 120));
  }
  delay(500);
  Serial.println("lewa w druga strone");
  //for (uint16_t pulselen = SERVOMAX; pulselen > SERVOMIN; pulselen--) {
    //servoLeft.write(map(pulselen, SERVOMIN, SERVOMAX, 0, 180));
  //}
  for (uint16_t pulselen = SERVOMIN; pulselen < SERVOMAX; pulselen++) {
    servoLeft.write(map(pulselen, SERVOMIN, SERVOMAX, 120, 60));
  }
*/


  /*
  Serial.println("lewa w jedna strone");
  for (uint8_t pos = 1; pos <= 179; pos++) {
    servoLeft.write(pos);
    delay(1);
  }
  delay(500);
  Serial.println("lewa w druga strone");
  for (uint8_t pos = 179; pos >= 1; pos--) {
    servoLeft.write(pos);
    delay(1);
  }

  delay(500);
  Serial.println("prawa w jedna strone");
  for (uint8_t pos = 1; pos <= 179; pos++) {
    servoRight.write(pos);
    delay(1);
  }
  delay(500);
  Serial.println("prawa w druga strone");
  for (uint8_t pos = 179; pos >= 1; pos--) {
    servoRight.write(pos);
    delay(1);
  }
  */

  /*
  delay(500);
  Serial.println("test");
  Serial.println("w jedna strone");
  Serial.println(servonum);
  for (uint16_t pulselen = SERVOMIN; pulselen < SERVOMAX; pulselen++) {
    servos.setPWM(servonum, 0, map(pulselen, SERVOMIN, SERVOMAX, SERVOMIN, SERVOMAX));
    delay(1);
  }
  delay(500);
  Serial.println("i w druga strone");
  for (uint16_t pulselen = SERVOMIN; pulselen < SERVOMAX; pulselen++) {
    servos.setPWM(servonum, 0, map(pulselen, SERVOMIN, SERVOMAX, SERVOMAX, SERVOMIN));
    delay(1);
  }

  delay(500);

  servonum++;
  if (servonum > 15) servonum = 0;
  */
}

// Odczytuje dane z modułu Bluetooth
void readBluetoothData() {
  if (btSerial.available()) {
    //Serial.print("BT: Odebrano dane. Pierwszy bajt = \"");
    receivedData = btSerial.readString();		// zapis serii odebranych bajtów do zmiennej
    //Serial.print(receivedData.charAt(0), 10);
    //Serial.println("\".");
  }
}

// Sprawdza stan baterii co około 10 sekund
void checkBatteryVoltageEveryTenSeconds() {
  timeNow = millis();
  if (timeNow - timeSaved >= 10000UL) {         // jeśli minęło więcej niż 10 sekund -> sprawdzenie napięcia baterii
    timeSaved = timeNow;
	checkBatteryState();
  }
}

// Sprawdza stan baterii
void checkBatteryState() {
  double batteryVoltage = readBatteryVoltage();
  //    Serial.println(batteryVoltage);
  if (batteryVoltage < BATTERY_CRITICAL) {
    battState = BatteryLow;
	Serial.println("Warning. Low battery voltage level.");
  }
  else if (batteryVoltage < BATTERY_MEDIUM) {
    Serial.println("Medium battery voltage level. It is advised to turn off the robot and start recharging.");
  }
}

// Przypisuje ostatnio odebraną komendę do zmiennej
void setLastCommandValue() {
  if ((state != Inactive) && (battState != BatteryLow)) {
    lastCommand = static_cast<RobotCommand>(receivedData.charAt(0));	// zrzutowanie pierwszego bajtu ostatnio odebranej serii na enum komendy robota
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
  case Calibrating:
    stateCalibrating();
    break;
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
	case Calibrate:
    standToCalibrate();
    state = Calibrating;
    break;
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
void stateCalibrating() {
  switch (lastCommand) {
    case Calibrating:
    stillCalibrating();
    break;
    default:
    calibratingToStand();
    state = Standing;
    break;
  }
}

// Kończy obsługę modułu Bluetooth oraz portu szeregowego
void stateInactive() {
  btSerial.end();
  Serial.end();
}

// Każe robotowi stanąć na kończynach, ustawia jego stan oraz wyświetla komunikat o zakończeniu inicjalizacji
void stateInitialising() {
  initialPosToStand();
  state = Standing;
  Serial.println("Done!");
}

// RUCHY ROBOTA:
void stillStand() {
  //
}

void standToFront() {
  //
}

void standToBack() {
  //
}

void standToLeft() {
  //
}

void standToRight() {
  //
}

void standToTurnLeft() {
  //
}

void standToTurnRight() {
  //
}

void standToCalibrate() {
  servoRight.write(map(SERVOMID, SERVOMIN, SERVOMAX, 120, 60));
  servoLeft.write(map(SERVOMID, SERVOMIN, SERVOMAX, 60, 120));
  for (uint8_t i = 0; i < 16; ++i) {
    servos.setPWM(i, 0, SERVOMID);
  }
}

void standToInitialPos() {
  //
}

void stillFront() {
  //
}

void frontToStand() {
  //
}

void stillBack() {
  //
}

void backToStand() {
  //
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
  //
}

void turningLeftToStand() {
  //
}

void stillTurningRight() {
  //
}

void turningRightToStand() {
  //
}

void stillCalibrating() {
  servoRight.write(map(SERVOMID, SERVOMIN, SERVOMAX, 120, 60));
  servoLeft.write(map(SERVOMID, SERVOMIN, SERVOMAX, 60, 120));
  for (uint8_t i = 0; i < 16; ++i) {
    servos.setPWM(i, 0, SERVOMID);
  }
}

void calibratingToStand() {
  //
}

void initialPosToStand() {
	/*for (pulselen = SERVOMIN; pulselen < SERVOMID; pulselen += 5) {
		servoRight.write(map(SERVOMID, SERVOMIN, SERVOMAX, 120, 60));
		servos.setPWM(SERVO_RIGHT_MID0NUM, 0, SERVOMID);
		servos.setPWM(SERVO_RIGHT_REAR0NUM, 0, SERVOMID);
		servoLeft.write(map(SERVOMID, SERVOMIN, SERVOMAX, 60, 120));
		servos.setPWM(SERVO_LEFT_MID0NUM, 0, SERVOMID);
		servos.setPWM(SERVO_LEFT_REAR0NUM, 0, SERVOMID);
		
		servos.setPWM(SERVO_RIGHT_FRONT1NUM, 0, map(pulselen, SERVOMIN, SERVOMAX, SERVOMIN_1, SERVOMAX_1));
		servos.setPWM(SERVO_RIGHT_MID1NUM, 0, map(pulselen, SERVOMIN, SERVOMAX, SERVOMAX_1, SERVOMIN_1));
		servos.setPWM(SERVO_RIGHT_REAR1NUM, 0, map(pulselen, SERVOMIN, SERVOMAX, SERVOMAX_1, SERVOMIN_1));
		servos.setPWM(SERVO_LEFT_FRONT1NUM, 0, map(pulselen, SERVOMIN, SERVOMAX, SERVOMAX_1, SERVOMIN_1));
		servos.setPWM(SERVO_LEFT_MID1NUM, 0, map(pulselen, SERVOMIN, SERVOMAX, SERVOMIN_1, SERVOMAX_1));
		servos.setPWM(SERVO_LEFT_REAR1NUM, 0, map(pulselen, SERVOMIN, SERVOMAX, SERVOMIN_1, SERVOMAX_1));
		
		servos.setPWM(SERVO_RIGHT_FRONT2NUM, 0, map(pulselen, SERVOMIN, SERVOMAX, SERVOMAX_2, SERVOMIN_2));
		servos.setPWM(SERVO_RIGHT_MID2NUM, 0, map(pulselen, SERVOMIN, SERVOMAX, SERVOMIN_2, SERVOMAX_2));
		servos.setPWM(SERVO_RIGHT_REAR2NUM, 0, map(pulselen, SERVOMIN, SERVOMAX, SERVOMIN_2, SERVOMAX_2));
		servos.setPWM(SERVO_LEFT_FRONT2NUM, 0, map(pulselen, SERVOMIN, SERVOMAX, SERVOMIN_2, SERVOMAX_2));
		servos.setPWM(SERVO_LEFT_MID2NUM, 0, map(pulselen, SERVOMIN, SERVOMAX, SERVOMAX_2, SERVOMIN_2));
		servos.setPWM(SERVO_LEFT_REAR2NUM, 0, map(pulselen, SERVOMIN, SERVOMAX, SERVOMAX_2, SERVOMIN_2));
	}*/
	for (uint8_t i = 0; i <= 50; ++i)
	{
		RightFrontBodyServo::setPosition(50);
		RightFrontHipServo::setPosition(i);
		RightFrontKneeServo::setPosition(i);
		RightMiddleBodyServo::setPosition(50);
		RightMiddleHipServo::setPosition(i);
		RightMiddleKneeServo::setPosition(i);
		RightRearBodyServo::setPosition(50);
		RightRearHipServo::setPosition(i);
		RightRearKneeServo::setPosition(i);
		LeftFrontBodyServo::setPosition(50);
		LeftFrontHipServo::setPosition(i);
		LeftFrontKneeServo::setPosition(i);
		LeftMiddleBodyServo::setPosition(50);
		LeftMiddleHipServo::setPosition(i);
		LeftMiddleKneeServo::setPosition(i);
		LeftRearBodyServo::setPosition(50);
		LeftRearHipServo::setPosition(i);
		LeftRearKneeServo::setPosition(i);
	}
}
// dorobić funkcje dla każdego serwa oraz funkcje na każdy poziom lub na każdą kończynę

