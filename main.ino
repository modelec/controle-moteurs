// rotary encoder avec interrupt
#define encodeur1_A 2 
#define encodeur1_B 3

#define moteur1_A 11
#define moteur1_B 12

int encodeur1_compteur = 0 ;
bool encodeur1_etatA ;
bool encodeur1_dernierEtatA ;

long unsigned encodeur1_tempsA ;

volatile int lastEncoded = 0;
volatile long encoderValue = 0;

void setup() {
  // Setup reception consigne

  // Setup encodeur
  pinMode(encodeur1_A,INPUT);
  pinMode(encodeur1_B,INPUT);
  // état de A au setup
  encodeur1_dernierEtatA = digitalRead(encodeur1_A);
  // memorisation du temps pour eviter des erreurs de changements d'etat
  encodeur1_tempsA = millis();
  //turn pullup resistor on
  digitalWrite(encodeur1_A, HIGH);
  digitalWrite(encodeur1_B, HIGH);

  // attachInterrupt(digitalPinToInterrupt(encodeur1_A), encodeur1_changementA, CHANGE);
  attachInterrupt(0, updateEncoder, CHANGE); 

  // Setup moteur
  // OUTPUT

  Serial.begin (115200);
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.println(encodeur1_compteur);
  // delay(1000); //just here to slow down the output, and show it will work  even during a delay
}

void encodeur_changementA(){
  // on mesure A
  encodeur1_etatA = digitalRead(encodeur1_A);
   
    // controle du temps pour eviter des erreurs 
    if( abs(millis() - encodeur1_tempsA) > 50 ){
      // Si B different de l'ancien état de A alors
      if(digitalRead(encodeur1_B) != encodeur1_dernierEtatA){
        encodeur1_compteur--;
      }
      else{
        encodeur1_compteur++;
      }
      // memorisation du temps pour A
      encodeur1_tempsA = millis();
    } 
    // memorisation de l'état de A
    encodeur1_dernierEtatA = encodeur1_etatA ;
    
    //affichage du compteur
    Serial.print("Compteur 1:");
    Serial.println(encodeur1_compteur);

}

void updateEncoder()
{
  int MSB = digitalRead(encodeur1_A); //MSB = most significant bit
  int LSB = digitalRead(encodeur1_B); //LSB = least significant bit

  int encoded = (MSB << 1) |LSB; //converting the 2 pin value to single number
  int sum  = (lastEncoded << 2) | encoded; //adding it to the previous encoded value

  if(sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) encoderValue ++;
  if(sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) encoderValue --;

  if(encoderValue == 1) {
    encodeur1_compteur ++;
  } else {
    encodeur1_compteur --;
  }

  lastEncoded = encoded; //store this value for next time
}
