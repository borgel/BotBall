// https://www.pololu.com/product/2135

const int pwmAPin = 18;
const int pwmBPin = 17;
const int pwmMirror = 16;
const int phaseAPin = 15;
const int phaseBPin = 18;

// single status LED
const int ledPin = 19;

// active low
const int ledRPin = 6;
const int ledGPin = 3;
const int ledBPin = 4;

IntervalTimer mirrorTimer;

void setup() {
  // status LED
  pinMode(ledPin, OUTPUT);
  
  pinMode(ledRPin, INPUT);
  pinMode(ledGPin, INPUT);
  pinMode(ledBPin, INPUT);
  
  // setup PWM and direction pins
  pinMode(phaseAPin, OUTPUT);
  pinMode(phaseBPin, OUTPUT);
  pinMode(pwmAPin, OUTPUT);
  pinMode(pwmBPin, OUTPUT);

  // 150000 = 0.15s
  //  10000 = 0.01s
  //   1000 = .001s
  mirrorTimer.begin(handleMirrorPwm, 100);
}

static volatile uint8_t mirrorPwmOC;

void loop() {
  digitalWrite(ledPin, LOW);

  mirrorPwmOC = 30;
  
  // rotate in one way
  // dir
  digitalWrite(phaseAPin, HIGH);
  digitalWrite(phaseBPin, LOW);

  //PWM
  // 8 bits
  analogWrite(pwmAPin, 255);
  analogWrite(pwmBPin, 64);
  analogWrite(pwmAPin, 1);

  delay(1000);
  digitalWrite(ledPin, HIGH);

  mirrorPwmOC = 200;

  // rotate the other way
  digitalWrite(phaseAPin, LOW);
  digitalWrite(phaseBPin, HIGH);
  
  analogWrite(pwmAPin, 64);
  analogWrite(pwmBPin, 255);
  analogWrite(pwmAPin, 250);

  delay(1000);

  /*
  if(mirrorWasHigh) {
    // low
    pinMode(pwmMirror, OUTPUT);
    digitalWrite(pwmMirror, LOW);
  }
  else {
    // "open drain" hi-z
    pinMode(pwmMirror, INPUT);
  }
  mirrorWasHigh = !mirrorWasHigh;
  */
}

#define MIRROR_COUNT_MAX 255
static volatile uint8_t mirrorCount = 0;
void handleMirrorPwm(void) {
  mirrorCount++;

  if(mirrorCount > mirrorPwmOC) {
    // "open drain" hi-z
    pinMode(pwmMirror, INPUT);
  } else {
    pinMode(pwmMirror, OUTPUT);
    digitalWrite(pwmMirror, LOW);
  }
}
