#define SAFETY_MAX_POSITIONING_DURATION_SECONDS 10

// put your setup code here, to run once:
#define MIN_DIGIT_VALUE 0
#define MAX_DIGIT_VALUE 9

#define STEPS_PER_SELECTOR_POSITION 100
#define STEPS_PER_DIGIT_POSITION 100

#define NB_DIGITS 4

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

  if(positionDelta == 0){return;}
  //on se base par rapport à la dernière position désirée pour ne pas cumuler les erreurs
  selectorMotorTargetSteps = selectorMotorTargetSteps + positionDelta * STEPS_PER_SELECTOR_POSITION;
  motorGotoPosition(SELECTOR_MOTOR_PIN_FORWARD, SELECTOR_MOTOR_PIN_REVERSE, selectorMotorActualSteps, selectorMotorTargetSteps);
  selectedDigit = digitToSelect;
}

void motorGotoPosition(int motorForwardPin, int motorReversePin, volatile int& actualSteps, int targetSteps) {
  int direction = getSign(targetSteps - actualSteps);

  //si aucun mouvement nécéssaire, on ne bouge pas
  if(direction == 0){return;}

  turnMotor(motorForwardPin, motorReversePin, direction);
  while (actualSteps * direction <= targetSteps * direction) {
    // Serial.print("GotoPosition : ");
    // Serial.print(actualSteps * direction);
    // Serial.print(" < ");
    // Serial.println(targetSteps * direction);
  }
  brakeMotor(motorForwardPin, motorReversePin);
  delay(200);
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

void displayCurrentDigits(int* digits, int nbDigits) {
    Serial.print("|");
  for (int currentDigitIndex = 0; currentDigitIndex < nbDigits; currentDigitIndex++) {
    Serial.print(digits[currentDigitIndex]);
    Serial.print("|");
  }
  Serial.println(" ");
}

void testPositions(int digitIndex, int* digitPositionsArray, int nbDigits) {
  int digitTarget = MIN_DIGIT_VALUE;
  while (digitTarget <= MAX_DIGIT_VALUE) {
    if (digitIndex < nbDigits - 1) {
      testPositions(digitIndex + 1, digitPositionsArray, nbDigits);
    }
    changeDigitTo(digitIndex, digitTarget);
    Serial.print("digit ");
    Serial.println(digitIndex);
    displayCurrentDigits(digitPositionsArray, nbDigits);
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

void setup() {
  // initialize serial communication at 9600 bits per second:
  Serial.begin(115200);

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
}

void loop() {
  Serial.println("---DEBUT---");

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

  
  // testPositions(0, digits, NB_DIGITS);

  Serial.println("---FIN---");
  switchOffMotor(SELECTOR_MOTOR_PIN_FORWARD, SELECTOR_MOTOR_PIN_REVERSE);
  switchOffMotor(DIGIT_MOTOR_PIN_FORWARD, DIGIT_MOTOR_PIN_REVERSE);
  while (true) {
  }
}
