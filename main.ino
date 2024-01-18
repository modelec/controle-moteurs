#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))

// pin
#define interruptPinRA 2
#define interruptPinRB 3
#define interruptPinLA 19
#define interruptPinLB 18

#define pinMotorLA 5
#define pinMotorLB 6
#define pinMotorRA 7
#define pinMotorRB 8

// tick / cm
#define tickcmR 41.8
#define tickcmL 41.9
//tick / rad horaire
#define tickZR_P 623
#define tickZL_N 619
//tick / rad trigo
#define tickZL_P 624
#define tickZR_N 616


// parametre K
// rotation
#define KRap 0.6
#define KRai 0.4
#define KRad 0
// distance
#define Kdp 0.8
#define Kdi 0.2
#define Kdd 0

// vitesse max
#define maxSpeed 200
#define minSpeed 50
#define maxAcc 10

#define maxIdT maxSpeed / Kdi

// compte codeur 
long countR = 0;
long countL = 0;
long prevCountR = 0;
long prevCountL = 0;

float dT = 0;
float IdT = 0;
float zT = 0;
float IzT = 0;
float cmdV = 0;
char cmd;
int cmdR;
int cmdL;
int prevCmdR;
int prevCmdL;
char streamChar[32] ;
int i;
int incomingByte = 0; // for incoming serial data

unsigned long previousMillis;
unsigned long chrono;

float Ez;

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
  
  pinMode(interruptPinRA, INPUT_PULLUP);
  pinMode(interruptPinRB, INPUT_PULLUP);
  pinMode(interruptPinLA, INPUT_PULLUP);
  pinMode(interruptPinLB, INPUT_PULLUP);
  
  
  attachInterrupt(digitalPinToInterrupt(interruptPinRA), interruptR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(interruptPinLA), interruptL, CHANGE);
  
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


  if( millis() - previousMillis >100){
    previousMillis = millis();
    // on fait le delta de chaque roue
    dR = (float)(countR - prevCountR); // (float)(dc);
    prevCountR = countR;
    dL = (float)(countL - prevCountL); // (float)(dc);
    prevCountL = countL;
    
    // on fait le delta de distance et d'angle
    dD = (dR/tickcmR+dL/tickcmL)/2;
    
    
    if(dR>0){
        dR = dR/tickZR_P;
    }else{
        dR = dR/tickZR_N;
    }
    
    if(dL>0){
        dL = dL/tickZL_P;
    }else{
        dL = dL/tickZL_N;
    }
    
    dZ = (dR -dL)/2;

    dT += dD;
    zT -= dZ;

    //Serial.print("dT ");
    //Serial.print(dT);
    //Serial.print("  :zT ");
    //Serial.print(zT*180/PI);

    //Serial.print("  countR : ");
    //Serial.print(countR);
    //Serial.print("  countL : ");
    //Serial.print(countL);

    Serial.print(cmdR);
    Serial.print(',');
    Serial.print(cmdL);
    Serial.print(',');
    Serial.print(dT);
    Serial.print(',');
    Serial.print(zT*180/PI);

    // si non action de mouvement
    if (cmd==' '){ 
        //Serial.print(":1");
        // on envoie un stop
        sendCmd(0, 0);
        prevCmdL = 0;
        prevCmdR = 0;
        cmdL = 0;
        cmdR = 0;

    }
    // recallage
    else if (cmd=='d'){
      IdT += dT;
      if (IdT > maxIdT) {
        IdT = maxIdT;
      }
      if (IdT < -maxIdT) {
        IdT = -maxIdT;
      }
      if ((dT < 1) && (dT > -1)) {
        IdT = 0;
      }
      cmdV = Kdp * dT + Kdi * IdT + Kdd * dD;
      cmdR  = (int)(cmdV*maxAcc);
      //cmdR = dT * maxAcc;
      // on plafone a la vitesse de commande
      if(cmdR>maxSpeed){
          cmdR = maxSpeed;
      }
      if(cmdR<-(maxSpeed)){
          cmdR = -(maxSpeed);
      }

      cmdL = cmdR;

      // on limite l'acceleration
      if ((abs(cmdR)-abs(prevCmdR))>maxAcc){
          cmdR = prevCmdR+ maxAcc* (cmdR-prevCmdR)/ abs(cmdR-prevCmdR);
      }
      if ((abs(cmdL)-abs(prevCmdL))>maxAcc){
          cmdL = prevCmdL+ maxAcc* (cmdL-prevCmdL)/ abs(cmdL-prevCmdL);
      }
      
      IzT += zT;
      if (abs(zT) < PI/40 ){
          Ez = KRap * zT;
          IzT = 0;
      }else{
        Ez = KRap * zT + KRai * IzT + KRad * (-dZ);
      }
      cmdR -= (int)(Ez*abs(cmdR));
      cmdL += (int)(Ez*abs(cmdL));   

      prevCmdR = cmdR;
      prevCmdL = cmdL;

      sendCmd(cmdL, cmdR);
    }

    else if (cmd=='r'){
      IzT += zT;
      if (abs(zT) < PI/40 ){
          cmdV = KRap * zT;
          IzT = 0;
      }else{
        cmdV = KRap * zT + KRai * IzT + KRad * (-dZ);
      }
      cmdL  = (int)(cmdV*maxSpeed);
      //cmdR = (int)(zT*maxSpeed);
      // on plafone a la vitesse de commande
      if(cmdL>maxSpeed){
          cmdL = maxSpeed;
      }
      if(cmdL<-(maxSpeed)){
          cmdL = -(maxSpeed);
      }
      cmdR = -cmdL;

      // on limite l'acceleration
      if ((abs(cmdR)-abs(prevCmdR))>maxAcc){
          cmdR = prevCmdR+ maxAcc* (cmdR-prevCmdR)/ abs(cmdR-prevCmdR);
      }
      if ((abs(cmdL)-abs(prevCmdL))>maxAcc){
          cmdL = prevCmdL+ maxAcc* (cmdL-prevCmdL)/ abs(cmdL-prevCmdL);
      }
      prevCmdR = cmdR;
      prevCmdL = cmdL;
    
      sendCmd(cmdL, cmdR);
    }

    Serial.println("");
  }
}

void decryptIncom(){
  bool neg;
  i = 0;
  cmd = streamChar[i];
  if (streamChar[i] == '0'){
    countR=0;
    countL=0;
    dT = 0;
    zT = 0;
  }

  if (streamChar[i] == 'd'){
    i = 2;
    dT = 0;
    if (streamChar[i] == '-'){
      neg = true;
      i++;
    }else neg = false;
    while (isDigit(streamChar[i])) {
      dT = dT * 10 + streamChar[i] - '0';
      i++;
    }
    if (neg) dT = -dT;
  }
  if (streamChar[i] == 'r'){
    i = 2;
    zT = 0;
    if (streamChar[i] == '-'){
      neg = true;
      i++;
    }else neg = false;
    while (isDigit(streamChar[i])) {
      zT = zT * 10 + streamChar[i] - '0';
      i++;
    }
    if (neg) zT = -zT;
    zT = zT*PI/180;
  }
}


void interruptR()
{
  if (digitalRead(interruptPinRA) == digitalRead(interruptPinRB)) {  
    countR--;
  }else{
    countR++;
  }
}

void interruptL()
{
  if (digitalRead(interruptPinLA) == digitalRead(interruptPinLB)) {  
    countL++;
  }else{
    countL--;
  }
}


void sendCmd(int cmdG,int cmdD){

  if (cmdD>0){
    analogWrite25k(pinMotorRA, cmdD);
    digitalWrite(pinMotorRB, LOW);
    //digitalWrite(pinMotorRA,HIGH);
    //digitalWrite(pinMotorRB,LOW);
  }
  if (cmdG>0){
    digitalWrite(pinMotorLA, LOW);
    analogWrite25k(pinMotorLB, cmdG);
    //digitalWrite(pinMotorLB,HIGH);
    //digitalWrite(pinMotorLA,LOW);
  }

  if(cmdD<0){
    digitalWrite(pinMotorRA, LOW);
    analogWrite25k(pinMotorRB, -cmdD);
    //digitalWrite(pinMotorRA,LOW);
    //digitalWrite(pinMotorRB,HIGH);
  }
  if (cmdG<0){
    analogWrite25k(pinMotorLA, -cmdG);
    digitalWrite(pinMotorLB, LOW);
    //digitalWrite(pinMotorLB,LOW);
    //digitalWrite(pinMotorLA,HIGH);
  }

  if(cmdD==0){
    //analogWrite(pinMotorRA, 0);
    //analogWrite(pinMotorRB, 0);
    digitalWrite(pinMotorRA,LOW);
    digitalWrite(pinMotorRB,LOW);
  }

  if(cmdG==0){
    //analogWrite(pinMotorLA, 0);
    //analogWrite(pinMotorLB, 0);
    digitalWrite(pinMotorLB,LOW);
    digitalWrite(pinMotorLA,LOW);  
  }

}
