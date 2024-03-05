#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))

// pin
#define interruptPinRA 2
#define interruptPinRB 3
#define interruptPinLA 19
#define interruptPinLB 18

#define pinMotorRA 5
#define pinMotorRB 6
#define pinMotorLA 7
#define pinMotorLB 8

// tick / mm
#define tickcmR 4.211
#define tickcmL 4.191

//tick / rad horaire
#define tickZR_N 644
#define tickZL_P 633
//tick / rad trigo
#define tickZR_P 639
#define tickZL_N 645

// parametre K
// rotation distance
#define KRap 1.3
#define KRai 0.4
// rotation
#define Kap 1
#define Kai 0.2
// distance
#define Kdp 1
#define Kdi 0

// correction gauche
#define KL 10

// vitesse max
#define maxSpeed 270
#define minSpeed 70
#define maxAcc 15

// String streamChar;
char streamChar[32] ;
int i;
int incomingByte = 0; // for incoming serial data
// position du robot
float X = 0;
float Y = 0;
float Z = 0;

// position de la cible
float Xt = 0;
float Yt = 0;
float Zt = 0;

// compte codeur
long countR = 0;
long countL = 0;
long prevCountR = 0;
long prevCountL = 0;

// rampte de vitesse
const float rankSpeed = 15;
float cmdSpeed;

// precision de position a atteindre
const float precisionPos = 5; //mm
const float precisionZ = PI/360; //0.5 deg

float IEz, Idc;
bool goBack = true;
int rotSigne = 0;
int loopCount;
char cmd;
int cmdR,cmdL;
int prevCmdR,prevCmdL;

// chronometrage
unsigned long interval = (unsigned long) 50;
unsigned long previousMillis;


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

  long countR = 0;
  long countL = 0;
  float X = 0;
  float Y = 0;
  float Z = 0;

  cmdSpeed = 0;

  attachInterrupt(digitalPinToInterrupt(interruptPinRA), interruptR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(interruptPinLA), interruptL, CHANGE);

  previousMillis = millis();
}

void loop() {
  // delta odometrie
  float dR, dL, dD, dZ;
  float ErrR, ErrL;
  // calcul cible
  float dc,Ddc, signe, zc;
  float Ez, DEz;

  // reply only when you receive data:
  if (Serial.available() > 0) {
    // read the incoming byte:
    incomingByte = Serial.read();
    streamChar[i] = (char)incomingByte;
    i++;
  }

  if (incomingByte == 10){
    incomingByte = 0;
    decryptIncom();
    i = 0;
    loopCount = 10;
  }


  if( millis() - previousMillis >10){
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

    // on projete sur X et Y
    X += dD * cos(Z + dZ/2);
    Y += dD * sin(Z + dZ/2);

    // on ajout a Z
    Z  += dZ;
    Ez = 0;
    if(Z > PI ){
      Z=Z-2*PI;
    }
    if(Z < -PI ){
      Z=Z+2*PI;
    }

    Serial.print(X);
    Serial.print(":");
    Serial.print(Y);
    Serial.print(":");
    Serial.print(Z*180/PI);

    // Serial.print(countL);
    // Serial.print(";");
    // Serial.print(countR);

    // si non action de mouvement
    if (cmd==' ') {
      //Serial.print(":1");
      // on envoie un stop
      sendCmd(0, 0);
    }
    //mouvement
    if (cmd=='G' or cmd=='T') {
      // distance à la cible
      dc = sqrt((Xt-X)*(Xt-X)+(Yt-Y)*(Yt-Y));
      Idc += dc;
      if (abs(dc) > precisionPos ) {
        Ddc = Kdp * dc;
        Idc = 0;
      }else {
        Ddc = Kdp * dc + Kdi * Idc;
      }

      cmdR  = (int)(Ddc*rankSpeed);

      // on plafone a la vitesse de commande
      if(cmdR>cmdSpeed) {
        cmdR = cmdSpeed;
      }
      if(cmdR<-(cmdSpeed)) {
        cmdR = -(cmdSpeed);
      }
      cmdL = cmdR;

      //On regarde si la cible est à gauche ou à droite du robot
      if(Y > Yt) {
        signe = -1;
      }else {
        signe = 1;
      }

      //On calcule l'angle a la cible
      zc = signe * acos((Xt-X)/dc);

      // on calcul d'angle entre le robot et la cible
      Ez = zc - Z;

      if(Ez > PI ) {
        Ez=Ez-2*PI;
      }
      if(Ez < -PI ) {
        Ez=Ez+2*PI;
      }

      // si on part en marche arriere
      if(abs(dc) > precisionPos ) {
        // si la différence est >90, on part en marche arriere
        if(abs(Ez)>(PI/2)) {
          cmdR  = -cmdR;
          // cmdL  = -cmdL;
          cmdL = cmdR;
          // on retourne l'angle de cible pour rester en marche arriere
          if(Ez<0){
            Ez = Ez+PI;
          }else{
            Ez = Ez-PI;
          }
        }
      }

      // calcul de correction d'angle a faire
      IEz += Ez;
      if (abs(Ez) > PI/20 ) {
        DEz = Kap * Ez;
        IEz = 0;
      }else {
        DEz = Kap * Ez + Kai * IEz;
      }

      // si on a un trop grand angle a faire, non transit
      if(abs(Ez) > PI/8 and cmd!='T') {
        // on ne fait que tourner
        cmdR = (int)(DEz*cmdSpeed);
        cmdL = (int)(-DEz*cmdSpeed);
      }else {
        cmdR += (int)(DEz*abs(cmdR));
        cmdL -= (int)(DEz*abs(cmdL));
      }

      // on limite l'acceleration
      if ((abs(cmdR)-abs(prevCmdR))>maxAcc) {
        cmdR = prevCmdR+ maxAcc* (cmdR-prevCmdR)/ abs(cmdR-prevCmdR);
      }
      if ((abs(cmdL)-abs(prevCmdL))>maxAcc) {
        cmdL = prevCmdL+ maxAcc* (cmdL-prevCmdL)/ abs(cmdL-prevCmdL);
      }
      prevCmdR = cmdR;
      prevCmdL = cmdL;

    //rotation
    }else if(cmd=='R') {
      dc=0;

      cmdR = 0;
      cmdL = 0;
      Ez = Zt - Z;
      // si on est pret de l'arrive, on se tourne dans la bonne direction
      if(Ez > PI ) {
        Ez=Ez-2*PI;
      }
      if(Ez < -PI ) {
        Ez=Ez+2*PI;
      }
      // si on change de sens, on raz l'integrale pour éviter de surtourner
      if (Ez>0) {
        if (rotSigne != 1) {
          rotSigne = 1;
          IEz=0;
        }
      }else {
        if (rotSigne != -1) {
          rotSigne = -1;
          IEz=0;
        }
      }
      // calcul de correction d'angle a faire
      IEz += Ez;
      if (abs(Ez) > PI/20 ) {
        DEz = KRap * Ez;
        IEz = 0;
      }else {
        DEz = KRap * Ez + KRai * IEz;
      }

      cmdR = (int)(DEz*cmdSpeed);

      if(cmdR>cmdSpeed) {
        cmdR = cmdSpeed;
      }
      if(cmdR<-(cmdSpeed)) {
        cmdR = -(cmdSpeed);
      }

      cmdL = -cmdR;

      if ((abs(cmdR)-abs(prevCmdR))>maxAcc) {
        cmdR = prevCmdR+ maxAcc* (cmdR-prevCmdR)/ abs(cmdR-prevCmdR);
      }
      if ((abs(cmdL)-abs(prevCmdL))>maxAcc) {
        cmdL = prevCmdL+ maxAcc* (cmdL-prevCmdL)/ abs(cmdL-prevCmdL);
      }

      // DEz - retard gauche
      cmdR = cmdR + KL * (-dR - dL)*cmdSpeed;
      // DEz + retard gauche
      cmdL = cmdL + KL * (-dR - dL)*cmdSpeed;

      prevCmdR = cmdR;
      prevCmdL = cmdL;
    } else {
      IEz = 0;
      Idc = 0;
    }
    /* moteur R : 1 > 64 > 127
    moteur L : 128 > 192 > 255 */
    if(cmdR>maxSpeed){
      cmdR = maxSpeed;
    } else if(cmdR<-maxSpeed){
      cmdR = -maxSpeed;
    }
    if(cmdL>maxSpeed){
      cmdL = maxSpeed;
    }else if(cmdL<-maxSpeed){
      cmdL = -maxSpeed;
    }

    // vitesse minimal pour la phase d'approche
    if (abs(cmdR) < minSpeed) cmdR = minSpeed* (cmdR)/ abs(cmdR);
    if (abs(cmdL) < minSpeed) cmdL = minSpeed* (cmdL)/ abs(cmdL);

    Serial.print(":");

    // si on est arrivé depuis assez longtemps
    if (loopCount<=0){
      cmd=' ';
      // on envoie un stop
      sendCmd(0, 0);
      // on raz l'integral
      IEz = 0;
      Idc = 0;
      Serial.print("1");

    // si on est proche du but, on continue et on réuit le compte
    }else if((abs(dc) < precisionPos*4 and (cmd=='G' or cmd=='T')) or ( cmd=='R' and  abs(Ez) < precisionZ*4 )){
      loopCount-=1;

      // on envoie les commande à la SB
      sendCmd(cmdL, cmdR);
      Serial.print("0");

    // si on n'est pas encore au but, on continue et on met le compte à 10
    }else{
      loopCount=10;
      // on envoie les commande à la SB
      sendCmd(cmdL, cmdR);
      // Serial.print("(");
      // Serial.print(cmdL);
      // Serial.print(" | ");
      // Serial.print(cmdR);
      // Serial.print(")");
      // transite on envoie le retour à 10 cm de la cible, avant de ralentir
      if(abs(dc) < 10 and cmd=='T'){
        Serial.print("1");
      }else{
        Serial.print("0");
      }
    }
    Serial.println("");
  }
}

void decryptIncom(){
  bool neg;
  i = 0;
  // goto destination ou transite destination
    // G:xxx:yyy
    // T:xxx:yyy
    if (streamChar[0] == 'G' or streamChar[0]=='T') {
        cmd = streamChar[0];
        i = 2;
        Xt = 0;

        while (isDigit(streamChar[i])) {
            Xt = Xt * 10 + streamChar[i] - '0';
            i++;
        }

        i++;
        Yt = 0;
        while (isDigit(streamChar[i])) {
            Yt = Yt * 10 + streamChar[i] - '0';
            i++;
        }
    }

    // rotation à l'angle
    // R:zzz
    if (streamChar[0] == 'R') {
        cmd = streamChar[0];
        i = 2;
        Zt = 0;
        if (streamChar[i] == '-'){
            neg = true;
            i++;
        }else neg = false;
        while (isDigit(streamChar[i])) {
            Zt = Zt * 10 + streamChar[i] - '0';
            i++;
        }
        if (neg) Zt = -Zt;
        Zt = Zt*PI/180;
    }

    // stop
    // 0
    if (streamChar[0] == '0') {
        cmdSpeed = 0;
    }

    // commande de vitesse
    // V:vvv
    if (streamChar[0] == 'V') {
        cmdSpeed = 0;
        i = 2;
        while (isDigit(streamChar[i])) {
            cmdSpeed = cmdSpeed * 10 + streamChar[i] - '0';
            i++;
        }
        if(cmdSpeed>maxSpeed) cmdSpeed = maxSpeed;
    }

    // set de la position
    // S:xxx:yyy:zzz
    if (streamChar[0] == 'S') {
        countR = 0;
        countL = 0;
        prevCountR = 0;
        prevCountL = 0;
        i = 2;
        X = 0;
        while (isDigit(streamChar[i])) {
            X = X * 10 + streamChar[i] - '0';
            i++;
        }
        Xt = X;
        i++;
        Y = 0;
        while (isDigit(streamChar[i])) {
            Y = Y * 10 + streamChar[i] - '0';
            i++;
        }
        Yt = Y;

        i++;

        Z = 0;
        if (streamChar[i] == '-'){
            neg = true;
            i++;
        }else neg = false;
        while (isDigit(streamChar[i])) {
            Z = Z * 10 + streamChar[i] - '0';
            i++;
        }
        if (neg) Z = -Z;
        Z = Z*PI/180;
        Zt = Z;
    }

    // set de la position sur X
    // X:xxx
    if (streamChar[0] == 'X') {
        i = 2;
        X = 0;
        while (isDigit(streamChar[i])) {
            X = X * 10 + streamChar[i] - '0';
            i++;
        }
        Xt = X;
    }

    // set de la position sur Y
    // Y:yyy
    if (streamChar[0] == 'Y') {
        i = 2;
        Y = 0;
        while (isDigit(streamChar[i])) {
            Y = Y * 10 + streamChar[i] - '0';
            i++;
        }
        Yt = Y;
    }

    // set de la position sur Z
    // Z:zzz
    if (streamChar[0] == 'Z') {
        i = 2;
        Z = 0;
        if (streamChar[i] == '-'){
            neg = true;
            i++;
        }else neg = false;
        while (isDigit(streamChar[i])) {
            Z = Z * 10 + streamChar[i] - '0';
            i++;
        }
        if (neg) Z = -Z;
        Z = Z*PI/180;
        Zt = Z;
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
