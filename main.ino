// rotary encoder avec interrupt
#define encodeur1_A 2 
#define encodeur1_B 3

#define moteur1_A 11
#define moteur1_B 12

#define TOTAL_NB_PAS 2400
// #define RAYON_CODEUSE 45 // En mm
#define RAYON_CODEUSE 35 // En mm (roue de moteur)
#define PI 3.1415926535897932384626433832795

bool encodeur1_etatA ;
bool encodeur1_dernierEtatA ;

long unsigned encodeur1_tempsA ;

volatile int last_encoded1 = 0;
volatile long encoder1_value = 0;

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
  attachInterrupt(1, updateEncoder, CHANGE);

  // Setup moteur
  // OUTPUT

  Serial.begin (115200);
}

void loop() {
  delay(1000);
  Serial.print(calculer_distance(encoder1_value));
  Serial.print(" | ");
  Serial.println(encoder1_value);
}

void updateEncoder() {
  int MSB = digitalRead(encodeur1_A); //MSB = most significant bit
  int LSB = digitalRead(encodeur1_B); //LSB = least significant bit

  int encoded = (MSB << 1) |LSB; //converting the 2 pin value to single number
  int sum  = (last_encoded1 << 2) | encoded; //adding it to the previous encoded value

  if(sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) encoder1_value ++;
  if(sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) encoder1_value --;

  last_encoded1 = encoded; //store this value for next time
}

double calculer_distance(long nb_pas) {
  return (2*PI*RAYON_CODEUSE*nb_pas)/TOTAL_NB_PAS;
}
