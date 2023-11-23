// rotary encoder avec interrupt
// Encodeur 1
#define encodeur1_A 2 
#define encodeur1_B 3
// Encodeur 2
#define encodeur2_A 18 
#define encodeur2_B 19

// Constantes
#define TOTAL_NB_PAS 2400
// #define RAYON_CODEUSE 45 // En mm
#define RAYON_CODEUSE 30 // En mm (roue de moteur)
#define ECART 124 // En mm (Ecart entre les deux roues)
#define PI 3.1415926535897932384626433832795

// Encodeur 1
bool encodeur1_etatA ;
bool encodeur1_dernierEtatA ;
long unsigned encodeur1_tempsA ;
volatile int last_encoded1 = 0;
volatile long encoder1_value = 0;
// Encodeur 2
volatile int last_encoded2 = 0;
volatile long encoder2_value = 0;

void setup() {
  // Setup reception consigne

  // Setup encodeur
  pinMode(encodeur1_A,INPUT);
  pinMode(encodeur1_B,INPUT);
  pinMode(encodeur2_A,INPUT);
  pinMode(encodeur2_B,INPUT);
  // état de A au setup
  encodeur1_dernierEtatA = digitalRead(encodeur1_A);
  // memorisation du temps pour eviter des erreurs de changements d'etat
  encodeur1_tempsA = millis();
  //turn pullup resistor on
  digitalWrite(encodeur1_A, HIGH);
  digitalWrite(encodeur1_B, HIGH);
  digitalWrite(encodeur2_A, HIGH);
  digitalWrite(encodeur2_B, HIGH);
  // Encodeur 1
  attachInterrupt(digitalPinToInterrupt(encodeur1_A), updateEncoder1, CHANGE); 
  attachInterrupt(digitalPinToInterrupt(encodeur1_B), updateEncoder1, CHANGE);
  // Encodeur 2
  attachInterrupt(digitalPinToInterrupt(encodeur2_A), updateEncoder2, CHANGE); 
  attachInterrupt(digitalPinToInterrupt(encodeur2_B), updateEncoder2, CHANGE);
  // Setup moteur
  // OUTPUT

  Serial.begin (115200);
}

void loop() {
  delay(1000);
  // Encodeur 1
  Serial.print("Encodeur 1 : ");
  Serial.print(calculer_distance(encoder1_value));
  Serial.print(" | ");
  Serial.println(encoder1_value);

  Serial.print("Encodeur 2 : ");
  Serial.print(calculer_distance(encoder2_value));
  Serial.print(" | ");
  Serial.println(encoder2_value);

  Serial.print("Angle : ");
  Serial.println(calculer_angle(calculer_distance(encoder1_value), calculer_distance(encoder2_value)));
}

void updateEncoder1() {
  int MSB = digitalRead(encodeur1_A); //MSB = most significant bit
  int LSB = digitalRead(encodeur1_B); //LSB = least significant bit

  int encoded = (MSB << 1) |LSB; //converting the 2 pin value to single number
  int sum  = (last_encoded1 << 2) | encoded; //adding it to the previous encoded value

  if(sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) encoder1_value ++;
  if(sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) encoder1_value --;

  last_encoded1 = encoded; //store this value for next time
}
void updateEncoder2() {
  int MSB = digitalRead(encodeur2_A); //MSB = most significant bit
  int LSB = digitalRead(encodeur2_B); //LSB = least significant bit

  int encoded = (MSB << 1) |LSB; //converting the 2 pin value to single number
  int sum  = (last_encoded2 << 2) | encoded; //adding it to the previous encoded value

  if(sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) encoder2_value --;
  if(sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) encoder2_value ++;

  last_encoded2 = encoded; //store this value for next time
}

double calculer_distance(long nb_pas) {
  return (2*PI*RAYON_CODEUSE*nb_pas)/TOTAL_NB_PAS;
}

double calculer_angle(double a, double b) {
  // return the angles in degrees
  return (b-a)/ECART*180/PI;
}
