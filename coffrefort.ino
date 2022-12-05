#include <Adafruit_SSD1306.h>

// ----- BEGIN ----- display specific section
#define SCREEN_WIDTH 128     // OLED display width, in pixels
#define SCREEN_HEIGHT 32     // OLED display height, in pixels
#define OLED_RESET -1        // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C  ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
// ----- END ----- display specific section

#define SAFETY_MAX_POSITIONING_DURATION_SECONDS 10
#define PAUSE_BETWEEN_POSITIONS_MS 50
// put your setup code here, to run once:
#define MIN_DIGIT_VALUE 0
#define MAX_DIGIT_VALUE 9

#define STEPS_PER_SELECTOR_POSITION 100
#define STEPS_PER_DIGIT_POSITION 100

#define NB_DIGITS 4
const int max_combinations = pow((MAX_DIGIT_VALUE - MIN_DIGIT_VALUE), NB_DIGITS + 1);
int current_combination = 1;

#define OPENING_SENSOR_PIN 7

int digits[NB_DIGITS] = { 0, 0, 0, 0 };

int selectedDigit = 0;

#define SELECTOR_MOTOR_SENSOR_PIN_INTERUPT 2
#define SELECTOR_MOTOR_SENSOR_PIN_FORWARD 8
#define SELECTOR_MOTOR_PIN_FORWARD 5
#define SELECTOR_MOTOR_PIN_REVERSE 6
int selectorMotorTargetSteps = 0;
volatile int selectorMotorActualSteps = 0;

#define DIGIT_MOTOR_SENSOR_PIN_INTERUPT 3
#define DIGIT_MOTOR_SENSOR_PIN_FORWARD 9
#define DIGIT_MOTOR_PIN_FORWARD 10
#define DIGIT_MOTOR_PIN_REVERSE 11
int digitMotorTargetSteps = 0;
volatile int digitMotorActualSteps = 0;

void setup() {
  // initialize serial communication at 9600 bits per second:
  // Serial.begin(115200);

  pinMode(OPENING_SENSOR_PIN, INPUT_PULLUP);

  pinMode(SELECTOR_MOTOR_SENSOR_PIN_INTERUPT, INPUT);
  pinMode(SELECTOR_MOTOR_SENSOR_PIN_FORWARD, INPUT);
  pinMode(SELECTOR_MOTOR_PIN_FORWARD, OUTPUT);
  pinMode(SELECTOR_MOTOR_PIN_REVERSE, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(SELECTOR_MOTOR_SENSOR_PIN_INTERUPT), updateSelectorMotorSteps, RISING);

  pinMode(DIGIT_MOTOR_SENSOR_PIN_INTERUPT, INPUT);
  pinMode(DIGIT_MOTOR_SENSOR_PIN_FORWARD, INPUT);
  pinMode(DIGIT_MOTOR_PIN_FORWARD, OUTPUT);
  pinMode(DIGIT_MOTOR_PIN_REVERSE, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(DIGIT_MOTOR_SENSOR_PIN_INTERUPT), updateDigitMotorSteps, RISING);

  display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS);
  display.clearDisplay();
}

/**
 * @returns -1 si négatif, +1 si positif, 0 sinon
 *
 */
int getSign(int number) {
  return int(number > 0) - (number < 0);
}

void changeDigitTo(int digitToMove, int numberToReach) {
  selectDigit(digitToMove);

  int positionDelta = numberToReach - digits[digitToMove];

  //on se base par rapport à la dernière position désirée pour ne pas cumuler les erreurs
  digitMotorTargetSteps = digitMotorTargetSteps + positionDelta * STEPS_PER_SELECTOR_POSITION;
  motorGotoPosition(DIGIT_MOTOR_PIN_FORWARD, DIGIT_MOTOR_PIN_REVERSE, digitMotorActualSteps, digitMotorTargetSteps);
  digits[digitToMove] = numberToReach;
}

void selectDigit(int digitToSelect) {
  int positionDelta = digitToSelect - selectedDigit;

  if (positionDelta == 0) { return; }
  //on se base par rapport à la dernière position désirée pour ne pas cumuler les erreurs
  selectorMotorTargetSteps = selectorMotorTargetSteps + positionDelta * STEPS_PER_SELECTOR_POSITION;
  motorGotoPosition(SELECTOR_MOTOR_PIN_FORWARD, SELECTOR_MOTOR_PIN_REVERSE, selectorMotorActualSteps, selectorMotorTargetSteps);
  selectedDigit = digitToSelect;
}

bool isOpened() {
  return digitalRead(OPENING_SENSOR_PIN) == LOW;
}

void motorGotoPosition(int motorForwardPin, int motorReversePin, volatile int& actualSteps, int targetSteps) {
  int direction = getSign(targetSteps - actualSteps);

  //si aucun mouvement nécéssaire, on ne bouge pas
  if (direction == 0) { return; }

  turnMotor(motorForwardPin, motorReversePin, direction);
  while (actualSteps * direction <= targetSteps * direction) {
    if (isOpened()) {
      brakeMotor(motorForwardPin, motorReversePin);
      return;
      }
    // Serial.print("GotoPosition : ");
    // Serial.print(actualSteps * direction);
    // Serial.print(" < ");
    // Serial.println(targetSteps * direction);
  }
  brakeMotor(motorForwardPin, motorReversePin);
  delay(PAUSE_BETWEEN_POSITIONS_MS);
}

void turnMotor(uint8_t motorForwardPin, uint8_t motorReversePin, int direction) {
  if (direction > 0) {
    digitalWrite(motorForwardPin, HIGH);
    digitalWrite(motorReversePin, LOW);
  } else {
    digitalWrite(motorForwardPin, LOW);
    digitalWrite(motorReversePin, HIGH);
  }
}

void brakeMotor(uint8_t motorForwardPin, uint8_t motorReversePin) {
  digitalWrite(motorForwardPin, HIGH);
  digitalWrite(motorReversePin, HIGH);
}

void switchOffMotor(uint8_t motorForwardPin, uint8_t motorReversePin) {
  digitalWrite(motorForwardPin, LOW);
  digitalWrite(motorReversePin, LOW);
}

void printSerialCurrentDigits(int* digits, int nbDigits) {
  Serial.print("|");
  for (int currentDigitIndex = 0; currentDigitIndex < nbDigits; currentDigitIndex++) {
    Serial.print(digits[currentDigitIndex]);
    Serial.print("|");
  }
  Serial.println(" ");
}

void displayCurrenDigits(int* digits, int nbDigits) {
  display.clearDisplay();
  display.setTextSize(2);  // Draw 2X-scale text
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);

  for (int currentDigitIndex = 0; currentDigitIndex < nbDigits; currentDigitIndex++) {
    display.print(digits[currentDigitIndex]);
    if (currentDigitIndex < nbDigits - 1) {
      display.setCursor(display.getCursorX() - 2, 0);
      display.print(".");
      display.setCursor(display.getCursorX() - 2, 0);
    }
  }

  float progress = (float)current_combination / (float)max_combinations;
  // Serial.println(progress);
  // Serial.println(current_combination);
  // Serial.println(max_combinations);
  display.drawRect(0, display.height() / 2, display.width(), display.height() / 2, SSD1306_WHITE);
  display.fillRect(0, display.height() / 2, ((float)display.width()) * progress, display.height() / 2, SSD1306_WHITE);
  // current_combination

  display.display();
}

void testPositions(int digitIndex, int* digitPositionsArray, int nbDigits) {
  int digitTarget = MIN_DIGIT_VALUE;
  while (digitTarget <= MAX_DIGIT_VALUE) {
    if (isOpened()) { return; }    
    changeDigitTo(digitIndex, digitTarget);
    if (digitIndex < nbDigits - 1) {
      testPositions(digitIndex + 1, digitPositionsArray, nbDigits);
    }
    current_combination++;
    displayCurrenDigits(digitPositionsArray, nbDigits);
    // printSerialCurrentDigits(digitPositionsArray, nbDigits);
    digitTarget = digitPositionsArray[digitIndex] + 1;
  }
  return;
}

void updateSelectorMotorSteps() {
  if (digitalRead(SELECTOR_MOTOR_SENSOR_PIN_FORWARD) > 0) {
    selectorMotorActualSteps++;
  } else {
    selectorMotorActualSteps--;
  }
}

void updateDigitMotorSteps() {
  if (digitalRead(DIGIT_MOTOR_SENSOR_PIN_FORWARD) > 0) {
    digitMotorActualSteps++;
  } else {
    digitMotorActualSteps--;
  }
}


void loop() {
  // Serial.println("---DEBUT---");
  displayCurrenDigits(digits, NB_DIGITS);
  // ---test methode turnMotor
  // turnMotor(SELECTOR_MOTOR_PIN_FORWARD, SELECTOR_MOTOR_PIN_REVERSE, 1);
  // Serial.println(selectorMotorActualSteps);
  // delay(500);
  // Serial.println(selectorMotorActualSteps);

  // turnMotor(DIGIT_MOTOR_PIN_FORWARD, DIGIT_MOTOR_PIN_REVERSE, -1);
  // Serial.println(digitMotorActualSteps);
  // delay(500);
  // Serial.println(digitMotorActualSteps);

  // ---test methode goto
  // Serial.println(selectorMotorActualSteps);
  // motorGotoPosition(SELECTOR_MOTOR_PIN_FORWARD, SELECTOR_MOTOR_PIN_REVERSE, selectorMotorActualSteps, 5000);
  // Serial.println(selectorMotorActualSteps);
  // delay(500);
  // Serial.println(selectorMotorActualSteps);

  // ---test methode selectDigit
  // Serial.println(selectedDigit);
  // selectDigit(0);
  // Serial.println(selectedDigit);
  // delay(200);
  // selectDigit(1);
  // Serial.println(selectedDigit);
  // delay(200);
  // selectDigit(2);
  // Serial.println(selectedDigit);
  // delay(200);
  // selectDigit(3);
  // Serial.println(selectedDigit);
  // delay(200);
  // selectDigit(0);
  // Serial.println(selectedDigit);
  // delay(200);

  // ---test methode changeDigitTo
  // displayCurrentDigits(digits, NB_DIGITS);
  // changeDigitTo(0, 5);
  // displayCurrentDigits(digits, NB_DIGITS);
  // changeDigitTo(0, 0);
  // displayCurrentDigits(digits, NB_DIGITS);
  // changeDigitTo(3, 9);
  // displayCurrentDigits(digits, NB_DIGITS);
  // changeDigitTo(3, 0);
  // displayCurrentDigits(digits, NB_DIGITS);


  testPositions(0, digits, NB_DIGITS);

  // Serial.println("---FIN---");
  switchOffMotor(SELECTOR_MOTOR_PIN_FORWARD, SELECTOR_MOTOR_PIN_REVERSE);
  switchOffMotor(DIGIT_MOTOR_PIN_FORWARD, DIGIT_MOTOR_PIN_REVERSE);
  while (true) {
  }
}
