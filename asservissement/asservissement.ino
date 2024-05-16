#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))

// pin
#define interruptionPinDA 2
#define interruptionPinDB 3
#define interruptionPinGA 19
#define interruptionPinGB 18

#define moteurPinDA 5
#define moteurPinDB 6
#define moteurPinGA 7
#define moteurPinGB 8

// tick / mm
#define tickmmD 4.211
#define tickmmG 4.191

//tick / rad horaire
#define tickZD_N 633
#define tickZG_P 638
//tick / rad trigo
#define tickZD_P 637
#define tickZG_N 632

// compte codeur 
long countR = 0;
long countL = 0;
long prevCountR = 0;
long prevCountL = 0;

float coeff = 1;
float P = 0.5;
float I = 0.3;
float D = -0.2;

float PErrR, PErrL;
long IErrR, IErrL;

char cmd = ' ';
int cmdVD;
int cmdVG;
int cmdD;
int cmdG;
int acc = 100;
char streamChar[32] ;
int i;
int incomingByte = 0; // for incoming serial data

int vitesse;
int angle;
int vitesseRotation;
bool previousSigneVitesse = 1;

int reducG = 0;
int reducD = 0;

// chronometrage
unsigned long previousMillis;
unsigned long chrono;

// PWM output @ 25 kHz, only on pins 2, 3, 5 and 6, 7, 8.
// Output value should be between 0 and 320, inclusive.
void analogWrite25k(int pin, int value) {
  if (value < 0) value = 0;
  if (value > 320) value = 320;
  pinMode(pin, OUTPUT);
    switch (pin) {
        case 2:
            sbi(TCCR3A, COM3B1);
            OCR3B = value;
            break;
        case 3:
            sbi(TCCR3A, COM3C1);
            OCR3C = value;
            break;
        case 5:
            sbi(TCCR3A, COM3A1);
            OCR3A = value;
            break;
        case 6:
            sbi(TCCR4A, COM4A1);
            OCR4A = value;
            break;
        case 7:
            sbi(TCCR4A, COM4B1);
            OCR4B = value;
            break;
        case 8:
            sbi(TCCR4A, COM4C1);
            OCR4C = value;
            break;
        default:
            // no other pin will work
            break;
    }
}

void setup() {
  // put your setup code here, to run once:
  
    // Configure Timer 3 for PWM @ 25 kHz.
  TCCR3A = 0;           // undo the configuration done by...
  TCCR3B = 0;           // ...the Arduino core library
  TCNT3  = 0;           // reset timer
  TCCR3A = _BV(COM3A1)  // non-inverted PWM on ch. A
          | _BV(COM3B1)  // same on ch; B
          | _BV(COM3C1)  // same on ch; C
          | _BV(WGM31);  // mode 10: ph. correct PWM, TOP = ICR1
  TCCR3B = _BV(WGM43)   // ditto
          | _BV(CS30);   // prescaler = 1
  ICR3   = 320;         // TOP = 320
    // Configure Timer 4 for PWM @ 25 kHz.
  TCCR4A = 0;           // undo the configuration done by...
  TCCR4B = 0;           // ...the Arduino core library
  TCNT4  = 0;           // reset timer
  TCCR4A = _BV(COM4A1)  // non-inverted PWM on ch. A
          | _BV(COM4B1)  // same on ch; B
          | _BV(COM4C1)  // same on ch; C
          | _BV(WGM41);  // mode 10: ph. correct PWM, TOP = ICR1
  TCCR4B = _BV(WGM43)   // ditto
          | _BV(CS40);   // prescaler = 1
  ICR4   = 320;         // TOP = 320

  Serial.begin(115200);  //Init the Serial baudrate
  
  pinMode(interruptionPinDA, INPUT_PULLUP);
  pinMode(interruptionPinDB, INPUT_PULLUP);
  pinMode(interruptionPinGA, INPUT_PULLUP);
  pinMode(interruptionPinGB, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(interruptionPinDA), interruptD, CHANGE);
  attachInterrupt(digitalPinToInterrupt(interruptionPinGA), interruptG, CHANGE);
  
  previousMillis = millis();
}

void loop() {
  // delta odometrie
  float dR, dL, dD, dZ;
	float ErrR, ErrL;
  
  // reply only when you receive data:
  if (Serial.available() > 0) {
    // read the incoming byte:
    incomingByte = Serial.read();
    streamChar[i] = (char)incomingByte;
    i++;
  }

  if (incomingByte == 10){
    incomingByte =0;
    decryptIncom();
    i = 0;
  }

  //Serial.print(cmdG);
  //Serial.print(",");
  //Serial.println(cmdD);

  if( millis() - previousMillis >100){
    previousMillis = millis();
    // Serial.print(cmd);
    // Serial.print(" | ");
    if (cmd == ' ') {
      cmdG = 0;
      cmdD = 0;
    } else {
      // on fait le delta de chaque roue
      dR = (float)(countR - prevCountR); // (float)(dc);
      prevCountR = countR;
      dL = (float)(countL - prevCountL); // (float)(dc);
      prevCountL = countL;

      // on fait le delta de distance et d'angle
      dD = (dR/tickmmD+dL/tickmmG)/2;
      

      if(dR>0){
        dR = dR/tickZD_P;
      }else{
        dR = dR/tickZD_N;
      }
      
      if(dL>0){
        dL = dL/tickZG_P;
      }else{
        dL = dL/tickZG_N;
      }

      dZ = (dR -dL)/2*10;
      // TODO : Rajouter coef en fonction de la vitesse pour le reducteur
      /*int t = (vitesse != 0) ? abs(vitesse) / vitesse : 0;
      if (dZ > 0) {
          if (abs(reducG) > 0) reducG -= t;
          else reducD += t;
      }
      if (dZ < 0) {
          if (abs(reducD) > 0) reducD -= t;
          else reducG += t;
      }
      cmdG = vitesse - reducG;
      cmdD = vitesse - reducD;*/
      cmdD = vitesse;
      cmdG = vitesse;

      if (angle != 0) {
        float ratio = abs(angle) / 157;
        if (angle > 0) {
          cmdD *= (1-ratio);
        }
        if (angle < 0) {
          cmdG *= (1-ratio);
        }
      }
      if (vitesseRotation != 0) {
        // TODO : Rajouter coef en fonction de la vitesse pour le reducteur
        cmdG = vitesseRotation;
        cmdD = -vitesseRotation;
      }
    }
    if (cmdG > 320) cmdG = 320;
    if (cmdD > 320) cmdD = 320;
    if (cmdG < -320) cmdG = -320;
    if (cmdD < -320) cmdD = -320;
    sendCmd(cmdG,cmdD);
  }

}

void decryptIncom() {
  bool neg;
  i = 0;

  if (streamChar[0] == 'W') {
    cmd = ' ';
    countL = 0;
    countR = 0;
    prevCountL = 0;
    prevCountR = 0;
    reducD = 0;
    reducG = 0;
    vitesse = 0;
    angle = 0;
    vitesseRotation = 0;
  }

  if (streamChar[0] == 'V') {
    cmd = streamChar[0];
    i = 2;
    vitesse = 0;
    
    if (streamChar[i] == '-'){
      neg = true;
      i++;
    }else neg = false;

    while (isDigit(streamChar[i])) {
      vitesse = vitesse * 10 + streamChar[i] - '0';
      i++;
    }

    if (neg) vitesse = -vitesse;

    /*if (previousSigneVitesse != abs(vitesse)/vitesse) {
      previousSigneVitesse != previousSigneVitesse;
      reducD = -reducD;
      reducG = -reducG;
    }*/
  }
  if (streamChar[0] == 'R') {
    cmd = streamChar[0];
    i = 2;
    vitesseRotation = 0;
    
    if (streamChar[i] == '-'){
      neg = true;
      i++;
    }else neg = false;

    while (isDigit(streamChar[i])) {
      vitesseRotation = vitesseRotation * 10 + streamChar[i] - '0';
      i++;
    }

    if (neg) vitesseRotation = -vitesseRotation;
  }
  if (streamChar[0] == 'A') {
    cmd = streamChar[0];
    i = 2;
    angle = 0;
    
    if (streamChar[i] == '-'){
      neg = true;
      i++;
    }else neg = false;

    while (isDigit(streamChar[i])) {
      angle = angle * 10 + streamChar[i] - '0';
      i++;
    }

    if (neg) angle = -angle;
  }
}

void interruptD() {
  if (digitalRead(interruptionPinDA) == digitalRead(interruptionPinDB)) {  
    countR--;
  }else{
    countR++;
  }
}
void interruptG() {
  if (digitalRead(interruptionPinGA) == digitalRead(interruptionPinGB)) {  
    countL++;
  }else{
    countL--;
  }
}

void sendCmd(int cmdG,int cmdD) {

  if (cmdD>0){
    analogWrite25k(moteurPinDA, cmdD);
    digitalWrite(moteurPinDB, LOW);
    //digitalWrite(moteurPinDA,HIGH);
    //digitalWrite(moteurPinDB,LOW);
  }
  if (cmdG>0){
    digitalWrite(moteurPinGA, LOW);
    analogWrite25k(moteurPinGB, cmdG);
    //digitalWrite(moteurPinGA,HIGH);
    //digitalWrite(moteurPinGB,LOW);
  }

  if(cmdD<0){
    digitalWrite(moteurPinDA, LOW);
    analogWrite25k(moteurPinDB, -cmdD);
    //digitalWrite(moteurPinDA,LOW);
    //digitalWrite(moteurPinDB,HIGH);
  }
  if (cmdG<0){
    analogWrite25k(moteurPinGA, -cmdG);
    digitalWrite(moteurPinGB, LOW);
    //digitalWrite(moteurPinGB,LOW);
    //digitalWrite(moteurPinGA,HIGH);
  }

  if(cmdD==0){
    //analogWrite(moteurPinDA, 0);
    //analogWrite(moteurPinDB, 0);
    digitalWrite(moteurPinDA,LOW);
    digitalWrite(moteurPinDB,LOW);
  }

  if(cmdG==0){
    //analogWrite(moteurPinGA, 0);
    //analogWrite(moteurPinGB, 0);
    digitalWrite(moteurPinGB,LOW);
    digitalWrite(moteurPinGA,LOW);  
  }

}
