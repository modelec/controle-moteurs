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
#define tickZD_N 636
#define tickZG_P 635
//tick / rad trigo
#define tickZD_P 636
#define tickZG_N 632

// parametre K
// distance
#define Kdp 0.5
#define Kdi 0.01
// rotation distance
#define Kap 1.3
#define Kai 0.005
// rotation
#define KRap 2.4
#define KRai 0.4

// correction gauche
#define KG 1

// vitesse max
#define maxSpeed 250
#define minSpeed 70
#define maxAcc 15
#define minSpeedPasInt 120

// String streamChar;
char streamChar[32] ;
int i;
int octetArrivant = 0; // for incoming serial data
// position du robot
float X = 0;
float Y = 0;
float Z = 0;

// position de la cible
float Xt = 0;
float Yt = 0;
float Zt = 0;

// compte codeur 
long compteD = 0;
long compteG = 0;
long comptePrecD = 0;
long comptePrecG = 0;

// rampte de vitesse
const float rankSpeed = 15;
float cmdVitesse;

// precision de position a atteindre
const float precisionPos = 8; //mm
const float precisionZ = PI/360; //0.5 deg

float IEz, Idc;
bool goBack = true;
int signeRot = 0;
int compteBoucle;
char cmd;
int cmdD,cmdG;
int cmdPrecD,cmdPrecG;

// chronometrage
unsigned long interval = (unsigned long) 50;
unsigned long millisPrecedente;


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

  Serial.begin(9600);  //Init the Serial baudrate
  
  pinMode(interruptionPinDA, INPUT_PULLUP);
  pinMode(interruptionPinDB, INPUT_PULLUP);
  pinMode(interruptionPinGA, INPUT_PULLUP);
  pinMode(interruptionPinGB, INPUT_PULLUP);
  
  long countD = 0;
  long countG = 0;
  float X = 0;
  float Y = 0;
  float Z = 0;
  
  cmdVitesse = 0;

  attachInterrupt(digitalPinToInterrupt(interruptionPinDA), interruptD, CHANGE);
  attachInterrupt(digitalPinToInterrupt(interruptionPinGA), interruptG, CHANGE);
  
  millisPrecedente = millis();
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
    octetArrivant = Serial.read();
    streamChar[i] = (char)octetArrivant;
    i++;
  }

  if (octetArrivant == 10){
    octetArrivant = 0;
    decryptIncom();
    i = 0;
    compteBoucle = 10;
  }


  if( millis() - millisPrecedente >10){
    millisPrecedente = millis();
    // on fait le delta de chaque roue
    dR = (float)(compteD - comptePrecD); // (float)(dc);
    comptePrecD = compteD;
    dL = (float)(compteG - comptePrecG); // (float)(dc);
    comptePrecG = compteG;
    
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
    
    dZ = (dR -dL)/2;

    // on projete sur X et Y
    X += dD * cos(Z + dZ/2);
    Y -= dD * sin(Z + dZ/2);

    // on ajout a Z
    Z  += dZ;
    Ez = 0;
    if(Z > PI ){
      Z=Z-2*PI;
    }
    if(Z < -PI ){
      Z=Z+2*PI;
    }

    // si non action de mouvement
    if (cmd==' ') {
      Serial.print(X);
      Serial.print(",");
      Serial.print(Y);
      Serial.print(",");
      Serial.print(Z*100);
      Serial.print(":1");
      // on envoie un stop
      sendCmd(0, 0);
    } else {
      Serial.print(X);
      Serial.print(",");
      Serial.print(Y);
      Serial.print(",");
      Serial.print(Z*100);
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

        cmdD  = (int)(Ddc*rankSpeed);

        // on plafone a la vitesse de commande
        if(cmdD>cmdVitesse) {
          cmdD = cmdVitesse;
        }
        if(cmdD<-(cmdVitesse)) {
          cmdD = -(cmdVitesse);
        }
        cmdG = cmdD;

        //On regarde si la cible est à gauche ou à droite du robot
        if(Y > Yt) {
          signe = 1;
        }else {
          signe = -1;
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
            cmdD  = -cmdD;
            // cmdL  = -cmdL;
            cmdG = cmdD;
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
        // Angle superieur à 9 degrés
        if (abs(Ez) > PI/20 ) {
          DEz = Kap * Ez;
          IEz = 0;
        }else {
          DEz = Kap * Ez + Kai * IEz;
        }
        
        // si on a un trop grand angle a faire, non transit (22,5 degrés)
        if(abs(Ez) > PI/8 and cmd!='T') {
          // on ne fait que tourner
          cmdD = (int)(DEz*cmdVitesse);
          cmdG = (int)(-DEz*cmdVitesse);
        }else {
          cmdD += (int)(DEz*abs(cmdD));
          cmdG -= (int)(DEz*abs(cmdG)); 
        }
        
        // on limite l'acceleration
        if ((abs(cmdD)-abs(cmdPrecD))>maxAcc) {
          cmdD = cmdPrecD+ maxAcc* (cmdD-cmdPrecD)/ abs(cmdD-cmdPrecD);
        }
        if ((abs(cmdG)-abs(cmdPrecG))>maxAcc) {
          cmdG = cmdPrecG+ maxAcc* (cmdG-cmdPrecG)/ abs(cmdG-cmdPrecG);
        }

        // DEz - retard gauche
        //cmdD = cmdD - KG * (dR + dL)*cmdVitesse;
        // DEz + retard gauche
        //cmdG = cmdG + KG * (dR + dL)*cmdVitesse;

        cmdPrecD = cmdD;
        cmdPrecG = cmdG;
        
      //rotation
      }else if(cmd=='R') {
        dc=0;
        
        cmdD = 0;
        cmdG = 0;
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
          if (signeRot != 1) {
            signeRot = 1;
            IEz=0;
          }
        }else {
          if (signeRot != -1) {
            signeRot = -1;
            IEz=0;
          }
        }
        // calcul de correction d'angle a faire (9)
        IEz += Ez;
        if (abs(Ez) > PI/20 ) {
          DEz = KRap * Ez;
          IEz = 0;
        }else {
          DEz = KRap * Ez + KRai * IEz;
        }
        
        cmdD = (int)(DEz*cmdVitesse);
        
        if(cmdD>cmdVitesse) {
          cmdD = cmdVitesse;
        }
        if(cmdD<-(cmdVitesse)) {
          cmdD = -(cmdVitesse);
        }
        
        cmdG = -cmdD;
        
        if ((abs(cmdD)-abs(cmdPrecD))>maxAcc) {
          cmdD = cmdPrecD+ maxAcc* (cmdD-cmdPrecD)/ abs(cmdD-cmdPrecD);
        }
        if ((abs(cmdG)-abs(cmdPrecG))>maxAcc) {
          cmdG = cmdPrecG+ maxAcc* (cmdG-cmdPrecG)/ abs(cmdG-cmdPrecG);
        }
        
        // DEz - retard gauche
        cmdD = cmdD + KG * (-dR - dL)*cmdVitesse;
        // DEz + retard gauche
        cmdG = cmdG + KG * (-dR - dL)*cmdVitesse;
        
        cmdPrecD = cmdD;
        cmdPrecG = cmdG;
      } else {
        IEz = 0;
        Idc = 0;
      }
      /* moteur R : 1 > 64 > 127
      moteur L : 128 > 192 > 255 */
      if(cmdD>maxSpeed){
        cmdD = maxSpeed;
      } else if(cmdD<-maxSpeed){
        cmdD = -maxSpeed;
      }
      if(cmdG>maxSpeed){
        cmdG = maxSpeed;
      }else if(cmdG<-maxSpeed){
        cmdG = -maxSpeed;
      }
      
      if ((abs(Ez) > PI/20) || (abs(dc) > precisionPos) ) {
        // vitesse minimal si pas d'intégrale (peut marcher meme si moteur initialement etein)
        if (abs(cmdD) < minSpeed) cmdD = minSpeedPasInt* (cmdD)/ abs(cmdD);
        if (abs(cmdG) < minSpeed) cmdG = minSpeedPasInt* (cmdG)/ abs(cmdG);
      } else {
        // vitesse minimal pour la phase d'approche
        if (abs(cmdD) < minSpeed) cmdD = minSpeed* (cmdD)/ abs(cmdD);
        if (abs(cmdG) < minSpeed) cmdG = minSpeed* (cmdG)/ abs(cmdG);
      }
      
      

      Serial.print(":");
          
      // si on est arrivé depuis assez longtemps
      if (compteBoucle<=0){
        cmd=' ';
        // on envoie un stop
        sendCmd(0, 0);
        // on raz l'integral
        IEz = 0;
        Idc = 0;
        Serial.print("1");

      // si on est proche du but, on continue et on réuit le compte
      }else if((abs(dc) < precisionPos*4 and (cmd=='G' or cmd=='T')) or ( cmd=='R' and  abs(Ez) < precisionZ*4 )){
        compteBoucle-=1;

        // on envoie les commande
        sendCmd(cmdG, cmdD);
        Serial.print("0");

      // si on n'est pas encore au but, on continue et on met le compte à 10
      }else{
        compteBoucle=10;
        // on envoie les commande
        sendCmd(cmdG, cmdD);
        // transite on envoie le retour à 10 cm de la cible, avant de ralentir
        if(abs(dc) < 10 and cmd=='T'){
          Serial.print("1");
        }else{
          Serial.print("0");
        }
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
        Zt = Zt/100;
    }
    
    // stop
    // 0
    if (streamChar[0] == '0') {
        cmdVitesse = 0;
    }
    
    // commande de vitesse
    // V:vvv
    if (streamChar[0] == 'V') {
        cmdVitesse = 0;
        i = 2;
        while (isDigit(streamChar[i])) {
            cmdVitesse = cmdVitesse * 10 + streamChar[i] - '0';
            i++;
        }
        if(cmdVitesse>maxSpeed) cmdVitesse = maxSpeed;
    }
    
    // set de la position
    // S:xxx:yyy:zzz
    if (streamChar[0] == 'S') {
        compteD = 0;
        compteG = 0;
        comptePrecD = 0;
        comptePrecG = 0;
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
        Z = Z/100;
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
        Z = Z/100;
        Zt = Z;
    }

    // TODO : if p : pong
    if (streamChar[0] == 'p') {
      Serial.println("pong");
      Serial.println("pong");
      Serial.println("pong");
      Serial.println("pong");
      Serial.println("pong");
    }
}


void interruptD()
{
  if (digitalRead(interruptionPinDA) == digitalRead(interruptionPinDB)) {  
    compteD--;
  }else{
    compteD++;
  }
}

void interruptG()
{
  if (digitalRead(interruptionPinGA) == digitalRead(interruptionPinGB)) {  
    compteG++;
  }else{
    compteG--;
  }
}


void sendCmd(int cmdG,int cmdD){

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
