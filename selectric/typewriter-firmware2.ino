#include <Arduino.h>
#include <EEPROM.h>

#define latchInterposer0Pin 24  // solenoids
#define latchInterposer1Pin 25
#define latchInterposer2Pin 26
#define latchInterposer3Pin 27
#define latchInterposer4Pin 28
#define latchInterposer5Pin 29
#define cycleBailPin 23
#define shiftBarPin 14
#define newlineBarPin 21
#define enterBarPin 17
#define backspaceBarPin 16
#define spaceBarPin 15
#define bellPin 22

#define senwf0 0  // active high
#define senwf1 1
#define senwf2 18
#define senwf3 3
#define senwf4 4
#define senwf5 5
#define senSpace 6  // active high
#define senBackspace 7
#define senReturn 8
#define senIndex 9
#define senShift 30   // active low
#define senKey 2     // active low
#define senExp 31     // active low
#define senMargin 20  // active high
#define senTab 19     // active low

#define rts 13
#define led 12

class PulseExtender {  // when tripped with reset(), active() will stay true until reset() has not been called for at least {duration}
  private:
    unsigned long now;
    unsigned long start;
    unsigned int duration;

  public:
    PulseExtender(unsigned int time) {
      duration = time;
    };
    void reset() {
      now = millis();
      start = now;
    }
    bool active() {
      now = millis();
      if (now - start >= duration) {
        return false;
      } else {
        return true;
      }
    }
    void expire() {
      now = millis();
      start = now - duration - 1;
    }
};
class Inputter {  // handles input photointerrupters
  private:
    unsigned long now;    // current time
    unsigned long start;  // time at start
    int pin;
    bool polarity;
    unsigned int reset_interval;
    bool state;      // current state of pin (as realtime as possible)
    bool lastState;  // thing to compare how the pin may have changed
    bool fired;

  public:
    bool debounced;                            // filted debounce
    Inputter(int p, bool o, unsigned int t) {  // define pin to use; set polarity to what the active state will be
      polarity = o;
      pin = p;
      reset_interval = t;
      // pinMode(pin, INPUT_PULLUP); //already do this in setup()
    };
    void update() {
      now = millis();
      state = digitalRead(pin);
      if (state == lastState) {
        start = now;  // refresh reset timer
        return;       // skip the rest
      }
      // if current state is polarity and last state was not polarity and timeout time is expired,
      //   instantly set the debounce to 1 (active)
      if (state == polarity) {
        if (lastState == !polarity) {
          start = now;
          debounced = 1;
          lastState = state;
        }
      } else {                                // current state is not polarity, but don't change it unless the time expires
        if (now - start >= reset_interval) {  // time expired
          debounced = 0;
          lastState = state;
        }
      }
    }
    bool single() {  // fires true once when called, on rising edge of debounced. Will not fire again until debounce goes false again
      update();
      if (debounced == 0) {
        fired = 0;
        return 0;
      } else {
        if (fired == 0) {
          fired = 1;
          return 1;
        }
        return 0;
      }
    }
    void discard() {
      fired = 1;
      debounced = 0;
    }
};

PulseExtender shiftHold(1000);
PulseExtender halfDuplex(500);
PulseExtender keyLock(20);
PulseExtender lightShow(50);
Inputter iSpace(senSpace, 1, 20);
Inputter iBackspace(senBackspace, 1, 20);
Inputter iReturn(senReturn, 1, 20);
Inputter iIndex(senIndex, 1, 20);
Inputter iMargin(senMargin, 1, 20);
Inputter iExp(senExp, 0, 20);
Inputter iTab(senTab, 0, 20);

const int eocd = 40;  // End Of char Delay (delay added after every cycle)
// the 128 possible print combinations. With bit 6 set, those are shifted characters.
// subbed ½ >- ~ ; ¼ -> | ; ¢ -> ^
// there are also patterns that produce the same char twice, this version replaces the duplicates with " ". The version that has the least number of solenoids pulled is used.
const char charmap[] = "-bw9    yhs0    qki6=n.2pe'5    jt~z    /lo4    ,ca8fuv3;dr7gxm1_BW(    YHS)    QKI^+N.@PE\"%    JT|Z    ?LO$    ,CA*FUV#:DR&GXM!";
//                      -bw9qki6yhs0pe'5qki6=n.2pe'5jt~zjt~z,ca8/1o4;dr7,ca8fuv3;dr7gxm1_BW(QKI^YHS)PE"%QKI^+N.@PE "%JT|ZJT|Z,CA*?LO$:DR&,CA*FUV#:DR&GXM! //RAW CHARS FROM BALL (TRY 2)
//abcdefghijklmnopqrstuvwxyz1234567890-=;',./~
//ABCDEFGHIJKLMNOPQRSTUVWXYZ!@#$%^&*()_+:"<>?|
//this one is for the reading of keys pressed. Extended keys are for when you press EXP, so all ascii characters can be typed
const char keymap[] = "-bw9qki6yhs0pe'5    =n.2    jt~z    ,ca8/lo4;dr7    fuv3    gxm1_BW(QKI^YHS)PE\"%    +N>@    JT|Z    <CA*?LO$:DR&    FUV#    GXM!\x1f\x02\x17{\x11\x0b\x09.\x19\x08\x13}\x10\x05`......\x0e\x1d\x00....\x0a\x14\\\x1a....\x1c\x03\x01.\x7f\x0c\x0f.\x1b\x04\x12.....\x06\x15\x16.....\x07\x18\x0d.\x1f\x02\x17{\x11\x0b\x09.\x19\x08\x13}\x10\x05`......\x0e\x1d\x00....\x0a\x14\\\x1a....\x1c\x03\x01.\x7f\x0c\x0f.\x1b\x04\x12.....\x06\x15\x16.....\x07\x18\x0d.";

bool keyReady = 0;
char keyValue = 0;
void readKeyISR() { //reads the letter keys when one is pressed
  if (!keyLock.active()) {
    uint8_t key = 0;
    key |= (digitalRead(senwf0) << 0) & 0b000001;
    key |= (digitalRead(senwf1) << 1) & 0b000010;
    key |= (digitalRead(senwf2) << 2) & 0b000100;
    key |= (digitalRead(senwf3) << 3) & 0b001000;
    key |= (digitalRead(senwf4) << 4) & 0b010000;
    key |= (digitalRead(senwf5) << 5) & 0b100000;
    keyLock.reset();
    if (digitalRead(senShift) == 0)key += 64;
    if (digitalRead(senExp) == 0)key += 128;
    keyReady = true;
    keyValue = keymap[key];
  }
}

void shiftOff();
int carriagePosition = 0;
int autoCR = 9999;

const int inBufSize = 64;
const int inBufClearReceiveFlagLimit = 40;
const int inBufSetReceiveFlagLimit = 10;
char inBuf[inBufSize];
int inBufreadPtr = 0;
int inBufwritePtr = 0;
bool receiveFlag = true;
int inBufFill() {  // reports the number of bytes left in the buffer
  if (inBufwritePtr >= inBufreadPtr) {
    return inBufwritePtr - inBufreadPtr;
  } else {
    return inBufSize - (inBufreadPtr - inBufwritePtr);
  }
}
void inBufManage() {  // call this often to buffer the Serial1 data
  if (Serial1.available()) {
    char v = Serial1.read();
    if (v == 24) {  // ascii cancel - empty buffer and stop immediately
      inBufwritePtr = 0;
      inBufreadPtr = 0;
      shiftOff();
    }
    inBuf[inBufwritePtr] = v;
    inBufwritePtr = (inBufwritePtr + 1) % inBufSize;
    if (inBufFill() >= inBufClearReceiveFlagLimit) {
      receiveFlag = false;
      digitalWrite(rts, HIGH);
    }
  }
}
char inBufGet() {  // reads a byte out of the Serial1 buffer
  if (inBufFill() <= inBufSetReceiveFlagLimit) {
    receiveFlag = true;
    digitalWrite(rts, LOW);
  }
  if (inBufreadPtr != inBufwritePtr) {
    char data = inBuf[inBufreadPtr];
    inBufreadPtr = (inBufreadPtr + 1) % inBufSize;
    return data;
  } else {
    return '\0';  // Return null character or any other suitable value
  }
}

void update() {  // polled items that run a lot all the time
  inBufManage();
  if (!shiftHold.active()) shiftOff();
  iSpace.update();
  iBackspace.update();
  iReturn.update();
  iIndex.update();
  iMargin.update();
  iExp.update();
  iTab.update();
  yield();
}

void delay2(unsigned long d) {  // delay, but do other things while we're waiting
  // uses micros, but is millis
  unsigned long start = micros();
  d *= 1000;
  unsigned long now = micros();
  while (now - start <= d) {
    now = micros();
    update();
  }
}

bool shiftCharState = false;  // keeps track of internal shift bar state
void shiftOn() {              // turns on and holds down shift bar. Stays down until shiftOff().
  shiftHold.reset();
  if (shiftCharState == true) return;
  shiftCharState = true;
  digitalWrite(shiftBarPin, HIGH);
  delay2(20);
  digitalWrite(shiftBarPin, LOW);
  analogWrite(shiftBarPin, 50);  // move to low power hold mode
  delay2(100);
}
void shiftOff() {  // releases a held shift bar
  if (shiftCharState == false) return;
  shiftCharState = false;
  analogWrite(shiftBarPin, 0);     // turn off low power mode
  digitalWrite(shiftBarPin, LOW);  // just in case
  delay2(100);
}

void charBackspace() {  // just pull the backspace bar
  digitalWrite(backspaceBarPin, HIGH);
  delay2(20);
  digitalWrite(backspaceBarPin, LOW);
  delay2(eocd * 3);
  carriagePosition--;
  if (carriagePosition <= 0)carriagePosition = 0;
}
void charSpace() {
  digitalWrite(spaceBarPin, HIGH);
  delay2(8);
  digitalWrite(spaceBarPin, LOW);
  delay2(eocd + 14);
  carriagePosition++;
}
void charTab() {              // tabs, as emulated by spaces
  const int spacesToTab = 4;  // this many spaces per tab
  int goToPosition = spacesToTab - (carriagePosition % spacesToTab);
  if (goToPosition == 0) goToPosition += 4;
  for (int i = 0; i < goToPosition; i++) {
    charSpace();
  }
}
void charCarriageReturnNewline() {
  if (carriagePosition == 0) {
    digitalWrite(newlineBarPin, HIGH);
    delay2(20);
    digitalWrite(newlineBarPin, LOW);
    delay2(110);
  } else {
    digitalWrite(enterBarPin, HIGH);
    
    delay2(20);
    digitalWrite(enterBarPin, LOW);
    delay2(carriagePosition * 7 + 20);
    carriagePosition = 0;
  }
}

bool balled(char k) {               // for chars we think might be mapped on the ball. Returns true if char was typed
  for (int i = 0; i <= 127; i++) {  // search through char map
    if (charmap[i] == k) {
      if (i & 0b1000000) {  // determine if shift bar should be pressed or not
        shiftOn();
      } else {
        shiftOff();
      }
      digitalWrite(cycleBailPin, 1);
      delay2(25);
      digitalWrite(cycleBailPin, 0);
      digitalWrite(latchInterposer0Pin, i & 0b000001);  // yes, the latch interposers move AFTER the cycle bail trips
      digitalWrite(latchInterposer1Pin, i & 0b000010);
      digitalWrite(latchInterposer2Pin, i & 0b000100);
      digitalWrite(latchInterposer3Pin, i & 0b001000);
      digitalWrite(latchInterposer4Pin, i & 0b010000);
      digitalWrite(latchInterposer5Pin, i & 0b100000);
      delay2(20);
      digitalWrite(latchInterposer0Pin, 0);
      digitalWrite(latchInterposer1Pin, 0);
      digitalWrite(latchInterposer2Pin, 0);
      digitalWrite(latchInterposer3Pin, 0);
      digitalWrite(latchInterposer4Pin, 0);
      digitalWrite(latchInterposer5Pin, 0);
      delay2(eocd);
      carriagePosition++;
      return 1;
      break;
    }
  }
  return 0;
}

void typeHandle(char k) {  // input an ascii value. Will make the typewriter type/do that action.
  if (k == 0) return;      // null
  if(carriagePosition>=autoCR){ //auto handle carriage return first
    charCarriageReturnNewline();
  }
  if (k == ' ') {          // handle space first
    charSpace();
    return;
  }
  if (balled(k)) {
    return;
  }
  switch (k) {  // other special chars
    default:    // unimplemented characters
      break;
    case '\r':  // newline+cr in linux
      charCarriageReturnNewline();
      break;
    case '\t':  // handle tabs later
      charTab();
      break;
    case 8:  // backspace
    case 127:
      charBackspace();
      break;
    case 11:  // vertical tab - used in place of newline
      digitalWrite(newlineBarPin, HIGH);
      delay2(20);
      digitalWrite(newlineBarPin, LOW);
      delay2(500);
      break;
    case 7:  // bell
      digitalWrite(bellPin, HIGH);
      delay2(20);
      digitalWrite(bellPin, LOW);
      break;
    case '<':  // less than sign
      balled('(');
      charBackspace();
      balled('V');
      break;
    case '>':  // greater than sign
      balled(')');
      charBackspace();
      balled('V');
      break;
    case '[':  // open square bracket
      balled('(');
      charBackspace();
      balled('S');
      break;
    case ']':  // close square bracket
      balled(')');
      charBackspace();
      balled('S');
      break;
    case '{':  // open curly brace
      balled('(');
      charBackspace();
      balled('C');
      break;
    case '}':  // close curly brace
      balled(')');
      charBackspace();
      balled('C');
      break;
    case '\\': {
        //      for(int i=0;i<=63;i++){  //type all characters on the ball
        //        digitalWrite(cycleBailPin, 1);
        //      delay(10);
        //      digitalWrite(cycleBailPin, 0);
        //      digitalWrite(latchInterposer0Pin, i & 0b000001);  // yes, the latch interposers move AFTER the cycle bail trips
        //      digitalWrite(latchInterposer1Pin, i & 0b000010);
        //      digitalWrite(latchInterposer2Pin, i & 0b000100);
        //      digitalWrite(latchInterposer3Pin, i & 0b001000);
        //      digitalWrite(latchInterposer4Pin, i & 0b010000);
        //      digitalWrite(latchInterposer5Pin, i & 0b100000);
        //      delay2(50);
        //      digitalWrite(latchInterposer0Pin, 0);
        //      digitalWrite(latchInterposer1Pin, 0);
        //      digitalWrite(latchInterposer2Pin, 0);
        //      digitalWrite(latchInterposer3Pin, 0);
        //      digitalWrite(latchInterposer4Pin, 0);
        //      digitalWrite(latchInterposer5Pin, 0);
        //      delay2(eocd+200);
        //      }
        balled('/');  // backslash
        charBackspace();
        balled('B');
      } break;
    case '`':  // backtick
      balled('\'');
      charBackspace();
      balled('B');
      break;
  }
}


PulseExtender echoTimer(200); //these help discard echoed characters so they don't get printed twice
char echoChar = 0;
void oval(char out) { //write to the serial port, but keep track of the last character for echo supression
  Serial1.write(out);
  echoTimer.reset();
  echoChar = out;
}

void setup() {
  Serial1.begin(1200);
  pinMode(latchInterposer0Pin, OUTPUT);
  pinMode(latchInterposer1Pin, OUTPUT);
  pinMode(latchInterposer2Pin, OUTPUT);
  pinMode(latchInterposer3Pin, OUTPUT);
  pinMode(latchInterposer4Pin, OUTPUT);
  pinMode(latchInterposer5Pin, OUTPUT);
  pinMode(shiftBarPin, OUTPUT);
  pinMode(cycleBailPin, OUTPUT);
  pinMode(bellPin, OUTPUT);
  pinMode(spaceBarPin, OUTPUT);
  pinMode(backspaceBarPin, OUTPUT);
  pinMode(enterBarPin, OUTPUT);
  pinMode(newlineBarPin, OUTPUT);
  pinMode(rts, OUTPUT);
  pinMode(led, OUTPUT);
  pinMode(senwf0, INPUT_PULLUP);
  pinMode(senwf1, INPUT_PULLUP);
  pinMode(senwf2, INPUT_PULLUP);
  pinMode(senwf3, INPUT_PULLUP);
  pinMode(senwf4, INPUT_PULLUP);
  pinMode(senwf5, INPUT_PULLUP);
  pinMode(senSpace, INPUT);
  pinMode(senBackspace, INPUT);
  pinMode(senReturn, INPUT);
  pinMode(senIndex, INPUT);
  pinMode(senShift, INPUT_PULLUP);
  pinMode(senKey, INPUT_PULLUP);
  pinMode(senExp, INPUT_PULLUP);
  pinMode(senMargin, INPUT_PULLUP);
  pinMode(senTab, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(senKey), readKeyISR, RISING);
  autoCR = EEPROM.read(32);
  if(autoCR < 1) autoCR = 255;
}

unsigned long next = 0;

void loop() {
  bool userModeNow = false;
  if (iMargin.debounced == 1)userModeNow = true;
//  if (iExp.debounced == 1)userModeNow = true;
  if (iMargin.debounced == 1  && iExp.debounced == 1){
    autoCR = carriagePosition;
    EEPROM.update(32, autoCR);
  }
  if (iMargin.debounced == 1  && digitalRead(senShift) == 0){
    autoCR = 255;
    EEPROM.update(32, autoCR);
  }
  char v = inBufGet();
  if (v != 0 && userModeNow == false) {
    if (echoTimer.active() && v == echoChar) { //echo supression. If last typed character is quickly received, discard it. If it's a different character, discard the echoChar
      v = 0;
      halfDuplex.expire();
    } else {
      echoChar = 0;
    }
    if (v != 0)halfDuplex.reset(); //do not immediately reset if in echo mode
    digitalWrite(led, LOW);
    typeHandle(v);
    keyReady = false;
  } else {
    if (!halfDuplex.active()) {
      shiftHold.expire();  // release shift if held
      if (userModeNow) {
        if (v != 0)lightShow.reset();
        if (lightShow.active()) {
          digitalWrite(led, v & 1);
        } else {
          digitalWrite(led, HIGH);
        }
      } else {
        digitalWrite(led, HIGH);
      }
      if (keyReady) { //from the key detection ISR
        keyReady = false;
        oval(keyValue);
        carriagePosition++;
      }
      if (iSpace.single()) {
        delay2(10);
        iSpace.discard();
        iBackspace.discard();
        iReturn.discard();
        iIndex.discard();
        oval(' ');
        carriagePosition++;
      }
      if (iBackspace.single()) {
        delay2(50);
        iSpace.discard();
        iBackspace.discard();
        iReturn.discard();
        iIndex.discard();
        oval(127); //need to send DEL to host for backspace
        carriagePosition--;
        if (carriagePosition <= 0)carriagePosition = 0;
      }
      if (iReturn.single()) {
        delay2(50);
        iSpace.discard();
        iBackspace.discard();
        iReturn.discard();
        iIndex.discard();
        delay2(carriagePosition * 7 + 20); //wait for carriage to go back to the beginning
        oval('\r');
        carriagePosition = 0;
      }
      if (iIndex.single()) {
        delay2(110);
        iSpace.discard();
        iBackspace.discard();
        iReturn.discard();
        iIndex.discard();
        oval('\n');
      }
      if (iTab.single()) {
        carriagePosition += 4;
        oval('\t');
      }
    }
  }
  update();

//    if (millis() >= next) {
//      next += 500;
//      Serial1.print(carriagePosition);
//      Serial1.print('\t');
//      Serial1.print(autoCR);
//      Serial1.println();
//    }

  //  delay(100);
  //  Serial1.print(digitalRead(senwf0));
  //  Serial1.print(digitalRead(senwf1));
  //  Serial1.print(digitalRead(senwf2));
  //  Serial1.print(digitalRead(senwf3));
  //  Serial1.print(digitalRead(senwf4));
  //  Serial1.print(digitalRead(senwf5));
  //  Serial1.println();
}
