#include <DueTimer.h> // Bibliothèque modifiée (pour compatibilité avec Servo): Timer0, 2,3,4 and 5 ne sont pas utilisables
#include <Servo.h>
#define TWOPI 6.283185307179586476925286766559 //2*PI
float rayon = 3.481; // rayon de la roue en cm - A VERIFIER!
float dist_roues = 26.1; // distance entre les roues en cm - A VERIFIER!
volatile long int ticks1 = 0; // Position du moteur en "ticks" (sachant qu'un tour contient 636 ticks). It will increment or decrement each time there is a change in the sensors
volatile long int ticks2 = 0; // Pulse counter variable. It will increment or decrement each time there is a change in the sensors
int ticks1_0; // position de référence lors de l'appel de tourner() / avancer()
int ticks2_0;
float Kp = 0.7; //Gain du correcteur proportionnel
int PWMmin = 55;
const float SOUND_SPEED = 340.0 / 1000; //vitesse du son / pour l'utilisation des capteurs US
const unsigned long MEASURE_TIMEOUT = 25000UL; //25ms
const byte ECHO3 = 45; //capteur US 3 // MUR
const byte TRIG3 = 43;
volatile byte state_det = LOW;
volatile byte obstacle = LOW;
const int pinLED = 23; // Définition pin pour la LED
const int pinCouleur = 25;
bool couleur_vert;
const int pinJack = 27;
volatile byte epreuve_en_cours = HIGH; //bit passant à "LOW" quand l'épreuve est terminée et impliquant des break à l'entrée des fonctions de déplacement


const int pinDIR1 = 12; //choix du sens de rotation du moteur 1 (HIGH / LOW)
const int pinPWM1 = 3; //choix de la vitesse de rotation (0 à 255)
const int pinBRAKE1 = 9; //pin sur HIGH pour activer les freins (HIGH / LOW)
const int pinSensorA1 = 50; //bleu  (2)
const int pinSensorB1 = 52; //violet   (4)

const int pinDIR2 = 13; //choix du sens de rotation du moteur 2
const int pinPWM2 = 11;
const int pinBRAKE2 = 8;
const int pinSensorA2 = 34; //bleu  (5)
const int pinSensorB2 = 32; //violet  (6)

Servo servo_catapulte;
Servo servo_bras_droit;
Servo servo_bras_gauche;
Servo servo_bras_abeille;


void setup() {
  // put your setup code here, to run once:
  pinMode(pinDIR1, OUTPUT);
  pinMode(pinPWM1, OUTPUT);
  pinMode(pinBRAKE1, OUTPUT);
  pinMode(pinSensorA1, INPUT_PULLUP); //INPUT_PULLUP : on s'intéresse ici au changement d'état de ces pins
  pinMode(pinSensorB1, INPUT_PULLUP);

  pinMode(pinDIR2, OUTPUT);
  pinMode(pinPWM2, OUTPUT);
  pinMode(pinBRAKE2, OUTPUT);
  pinMode(pinSensorA2, INPUT_PULLUP);
  pinMode(pinSensorB2, INPUT_PULLUP);

  pinMode(pinLED, OUTPUT);
  pinMode(pinCouleur, INPUT);
  pinMode(pinJack, INPUT);

  servo_catapulte.attach(10);
  servo_bras_droit.attach(7);
  servo_bras_gauche.attach(6);
  servo_bras_abeille.attach(5);

  Serial.begin(9600); // communication série avec l'ordi

  digitalWrite(pinBRAKE1, LOW);
  digitalWrite(pinBRAKE2, LOW);

  attachInterrupt(digitalPinToInterrupt(pinSensorA1), sensorAInterrupt1, CHANGE); //ces interruptions mettent à jour la position de chaque moteur
  attachInterrupt(digitalPinToInterrupt(pinSensorB1), sensorBInterrupt1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(pinSensorA2), sensorAInterrupt2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(pinSensorB2), sensorBInterrupt2, CHANGE);

  Timer1.attachInterrupt(isr_US_robot).start(100000);
  int duree_epreuve_en_sec = 10;
  Timer6.attachInterrupt(isr_fin_epreuve).start(duree_epreuve_en_sec*1000000); // fin de l'épreuve (à 10sec)

  //________________________________________________________

  digitalWrite(pinDIR1, HIGH);
  digitalWrite(pinDIR2, LOW);
}

void loop() {
    if (digitalRead(pinCouleur) == HIGH) // On définit la couleur du robot
  {
    couleur_vert = true;
  }
  else {
    couleur_vert = false;
  }

  if (digitalRead(pinJack) == LOW) // On scrute pour savoir quand démarrer
  {
    if (couleur_vert) // On entre dans la boucle VERTE
    {
      digitalWrite(pinLED, HIGH); // On allume la LED
      avancer_Kp(10000);
    }
    else // On entre dans la boucle ORANGE
    {
      digitalWrite(pinLED, LOW); // On éteint la LED
     // LOOP
    }
  }
}

//_________________________________________________________

void isr_fin_epreuve()
{
  epreuve_en_cours = LOW;
}

void sensorAInterrupt1()
{
  // Called when encoder A changes its value
  if (digitalRead(pinSensorA1)^digitalRead(pinSensorB1)) {
    ticks1--;
  }
  else ticks1++;

}

void sensorBInterrupt1()
{
  // Called when encoder B changes its value
  if (digitalRead(pinSensorA1)^digitalRead(pinSensorB1)) {
    ticks1++;
  }
  else ticks1--;

}

void sensorAInterrupt2()
{
  // Called when encoder A changes its value
  if (digitalRead(pinSensorA2)^digitalRead(pinSensorB2)) {
    ticks2--;
  }
  else ticks2++;
}

void sensorBInterrupt2()
{
  // Called when encoder B changes its value
  if (digitalRead(pinSensorA2)^digitalRead(pinSensorB2)) {
    ticks2++;
  }
  else ticks2--;
}

//________________________________________________________

void test_det(void) {
  if (state_det == HIGH)
  {
    digitalWrite(TRIG3, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG3, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG3, LOW);

    long measure = pulseIn(ECHO3, HIGH, MEASURE_TIMEOUT);
    float distance_mm = measure / 2.0 * SOUND_SPEED;

    //Serial.println("distance =");
    //Serial.println(distance_mm);
    
    obstacle = LOW;


    if (distance_mm < 150 && distance_mm > 5)
    {
      obstacle = HIGH;
    }

    //Serial.println("obstacle");
    //Serial.println(obstacle);

    state_det = LOW;
  }
}

void avancer_Kp(float distance_cm)
{
  digitalWrite(pinBRAKE1, LOW);
  digitalWrite(pinBRAKE2, LOW);

  if (distance_cm > 0)
  {
    digitalWrite(pinDIR1, HIGH); // à vérifier, il faut être sûr d'avancer et non pas de reculer!
    digitalWrite(pinDIR2, LOW);
  }

  else
  {
    digitalWrite(pinDIR1, LOW); // à vérifier, il faut être sûr d'avancer et non pas de reculer!
    digitalWrite(pinDIR2, HIGH);
  }

  int distance_ticks = floor((distance_cm) * 636.0 / (TWOPI * rayon)) ;
  ticks1_0 = ticks1;
  ticks2_0 = ticks2;

  while (abs(distance_ticks) - abs(ticks1 - ticks1_0) > 0)
  {
    if (epreuve_en_cours == LOW) break; // fin de l'épreuve
    test_det();
    if (obstacle == LOW)
    {
    digitalWrite(pinBRAKE1, LOW);
    digitalWrite(pinBRAKE2, LOW);

    int erreur = abs(distance_ticks) - abs(ticks1 - ticks1_0);

    //Serial.println("erreur");
    //Serial.println(erreur);

    if (Kp * erreur > 255)
    {
      analogWrite(pinPWM1, 255);
      analogWrite(pinPWM2, 255);
      //synchro_esclave(ticks1, 255);

    }
    else
    {
      int consigne = Kp * erreur + PWMmin; // (+PWMmin)
      analogWrite(pinPWM1, consigne);
      analogWrite(pinPWM2, consigne);
      //synchro_esclave(ticks1, consigne);
    }

    }

    if (obstacle == HIGH)
    {
    analogWrite(pinPWM1, 0);
    analogWrite(pinPWM2, 0);
    digitalWrite(pinBRAKE1, HIGH);
    digitalWrite(pinBRAKE2, HIGH);
    }

  }
  analogWrite(pinPWM1, 0);
  analogWrite(pinPWM2, 0);
  digitalWrite(pinBRAKE1, HIGH);
  digitalWrite(pinBRAKE2, HIGH);
}

void isr_US_robot()
{
  state_det = HIGH;
}
