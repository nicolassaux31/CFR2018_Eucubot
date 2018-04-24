/* Code Robot Eucubot CFR 2018 */

// Bibliothèques
#include <DueTimer.h> // Bibliothèque modifiée (pour compatibilité avec Servo): Timer0, 2,3,4 and 5 ne sont pas utilisables 
#include <Servo.h>

// Définition des constantes utiles
#define TWOPI 6.283185307179586476925286766559

float rayon = 3.481; // rayon de la roue en cm - A VERIFIER!
float dist_roues = 23.2; // distance entre les roues en cm - A VERIFIER!

const float SOUND_SPEED = 340.0 / 1000; //vitesse du son
const unsigned long MEASURE_TIMEOUT = 25000UL; //25ms

volatile long int ticks1 = 0; // Pulse counter variable. It will increment or decrement each time there is a change in the sensors
volatile long int ticks2 = 0; // Pulse counter variable. It will increment or decrement each time there is a change in the sensors

int ticks1_0; // position de référence lors de l'appel de tourner() / avancer()
int ticks2_0;

float Kp = 0.7;
//float Ki =0.05;
//int sum_err = 0;

float delta_655 = 3.7;

int PWM_min = 55; // prise en compte du seuil de démarrage des moteurs

// Configuration des PINS

const int pinDIR1 = 12;
const int pinPWM1 = 3;
const int pinBRAKE1 = 9;
const int pinSensorA1 = 50; //bleu  (2)
const int pinSensorB1 = 52; //violet   (4)

const int pinDIR2 = 13;
const int pinPWM2 = 11;
const int pinBRAKE2 = 8;
const int pinSensorA2 = 34; //bleu  (5)
const int pinSensorB2 = 32; //violet  (6)

const int pinLED = 23; // Définition pin pour la LED
// VCC+ moteur : rouge
// GRND moteur : noir
// VCC+ capteur : marron
// GRND capteur : vert

const byte TRIG1 = 10; //capteur US 1
const byte ECHO1 = 2;

volatile byte state = HIGH; //bit passant à "LOW" quand l'épreuve est terminée et impliquant des break à l'entrée des fonctions de déplacement
volatile byte state_det = LOW; //bit passant à HIGH lorsqu'on lit la valeur du capteur US associé au robot adverse
volatile byte obstacle = LOW; //bit passant à HIGH lorqu'un obstacle est détecté
volatile byte state_det_TOR = HIGH;
volatile byte TOR = LOW;

// PIN utilisables pour servo : 10 7 6 5 4 2
// Définition des servomoteurs
Servo servo_catapulte;
Servo servo_bras_droit;
Servo servo_bras_gauche;
Servo servo_bras_abeille;

// Définition pin pour la couleur vert/orange
const int pinCouleur = 25;

// Définition couleur séléctionnée

bool couleur_vert;

// Définition pin pour le démarrage du robot
const int pinJack = 27;

/*_____________________________________________________________________________________*/

void setup() {
  // Setup LED
  pinMode(pinLED, OUTPUT);

  // Définition de la couleur du robot
  pinMode(pinCouleur, INPUT);
  // Définition du démarrage
  pinMode(pinJack, INPUT);

  // Attache des servomoteurs
  servo_catapulte.attach(10);
  servo_bras_droit.attach(7);
  servo_bras_gauche.attach(6);
  servo_bras_abeille.attach(5);

  // put your setup code here, to run once:
  pinMode(pinDIR1, OUTPUT);
  pinMode(pinPWM1, OUTPUT);
  pinMode(pinBRAKE1, OUTPUT);
  pinMode(pinSensorA1, INPUT_PULLUP);
  pinMode(pinSensorB1, INPUT_PULLUP);

  pinMode(pinDIR2, OUTPUT);
  pinMode(pinPWM2, OUTPUT);
  pinMode(pinBRAKE2, OUTPUT);
  pinMode(pinSensorA2, INPUT_PULLUP);
  pinMode(pinSensorB2, INPUT_PULLUP);

  Serial.begin(9600);
  pinMode(TRIG1, OUTPUT);
  digitalWrite(TRIG1, LOW);
  pinMode(ECHO1, LOW);


  digitalWrite(pinBRAKE1, LOW);
  digitalWrite(pinBRAKE2, LOW);

  // Interruptions

  Timer1.attachInterrupt(isr_US_robot).start(100000); //timer pour la scrutation du capteur US (temps en microsecondes)
  Timer6.attachInterrupt(isr_fin_epreuve).start(100000000); // fin de l'épreuve (à 100sec)
  Timer7.attachInterrupt(isr_US_TOR).start(100000); //timer pour la scrutation des capteur US pour le tout ou rien (détection des réservoirs) (temps en microsecondes)

  attachInterrupt(digitalPinToInterrupt(pinSensorA1), sensorAInterrupt1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(pinSensorB1), sensorBInterrupt1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(pinSensorA2), sensorAInterrupt2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(pinSensorB2), sensorBInterrupt2, CHANGE);

  Serial.begin(9600);

  /*_____________________________________________________________________________________*/

}

void loop() {
  if (digitalRead(pinCouleur) == HIGH) // On définit la couleur du robot
  {
    couleur_vert = true;
  }
  else {
    couleur_vert = false;
  }

  if (couleur_vert) // On scrute pour savoir quand démarrer
  {
    if (digitalRead(pinCouleur) == HIGH) // On entre dans la boucle VERTE
    {
      digitalWrite(pinLED, HIGH); // On allume la LED
      avancer_Kp(49); // On se dirige vers le récupérateur n°1
      delay(1000);
      servo_bras_droit.write(0); // On place l'aiguillage pour aller vers la catapulte
      delay(1000);
      salve_catapulte(); // On fait tourner le servo pour projeter les balles
      delay(1000);
      avancer_Kp(87); // On avance jusqu'à l'abeille
      delay(1000);
      servo_bras_abeille.write(0); // On déploie le bras une première fois
      delay(1000);
      tourner_Kp(90); // On tourne le robot pour pousser l'abeille
      delay(1000);
      servo_bras_abeille.write(30); // On baisse le bras une seconde fois
      delay(1000);
      servo_bras_droit.write(0); // On place l'aiguillage pour aller vers le bras
      delay(1000);
      avancer_Kp(33); // On se dirige vers le récupérateur n°2
      delay(5000); // Dépose des balles
      servo_bras_abeille.write(0); // On range le bras
      delay(1000);
      tourner_Kp(90); // On place l'avant du robot vers l'interrupteur
      delay(1000);
      avancer_Kp(159); // On traverse le terrain (on suit la ligne noire)
      delay(1000);
      tourner_Kp(-90); // On tourne pour continuer de suivre la ligne
      delay(1000);
      avancer_Kp(52); // On avance au niveau de l'interrupteur
      delay(1000);
      tourner_Kp(90); // On se place en face de l'interrupteur
      delay(1000);
      avancer_Kp(35); // On fonce dans l'interrupteur et c'est FINI !
    }
    else // On entre dans la boucle ORANGE
    {
      digitalWrite(pinLED, LOW); // On éteint la LED
      avancer_Kp(49); // On se dirige vers le récupérateur n°1
      delay(1000);
      servo_bras_gauche.write(0); // On place l'aiguillage pour aller vers la catapulte
      delay(1000);
      salve_catapulte(); // On fait tourner le servo pour projeter les balles
      delay(1000);
      avancer_Kp(87); // On avance jusqu'à l'abeille
      delay(1000);
      servo_bras_abeille.write(0); // On déploie le bras une première fois
      delay(1000);
      tourner_Kp(-90); // On tourne le robot pour pousser l'abeille
      delay(1000);
      servo_bras_abeille.write(30); // On baisse le bras une seconde fois
      delay(1000);
      servo_bras_gauche.write(0); // On place l'aiguillage pour aller vers le bras
      delay(1000);
      avancer_Kp(33); // On se dirige vers le récupérateur n°2
      delay(5000); // Dépose des balles
      servo_bras_abeille.write(0); // On range le bras
      delay(1000);
      tourner_Kp(-90); // On place l'avant du robot vers l'interrupteur
      delay(1000);
      avancer_Kp(159); // On traverse le terrain (on suit la ligne noire)
      delay(1000);
      tourner_Kp(90); // On tourne pour continuer de suivre la ligne
      delay(1000);
      avancer_Kp(52); // On avance au niveau de l'interrupteur
      delay(1000);
      tourner_Kp(90); // On se place en face de l'interrupteur
      delay(1000);
      avancer_Kp(35); // On fonce dans l'interrupteur et c'est FINI !
    }
  }
  //avancer_Kp(20);
  //tourner_Kp(TWOPI/2);

}

/*_____________________________________________________________________________________*/
// Définition des fonctions :


// Définition des routines d'interruptions


void isr_US_robot()
{
  state_det = HIGH;
}

void isr_US_TOR()
{
  state_det_TOR = HIGH;
}

void test_det_TOR(void)
{
  if (state_det_TOR == HIGH);
  {
    digitalWrite(TRIG1, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG1, LOW);

    long measure = pulseIn(ECHO1, HIGH, MEASURE_TIMEOUT);
    float distance_mm = measure / 2.0 * SOUND_SPEED;

    TOR = LOW;

    if (distance_mm < 150)
    {
      TOR = HIGH;
    }
    state_det_TOR = LOW;
  }
}

void test_det(void) {
  if (state_det == HIGH)
  {
    digitalWrite(TRIG1, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG1, LOW);

    long measure = pulseIn(ECHO1, HIGH, MEASURE_TIMEOUT);
    float distance_mm = measure / 2.0 * SOUND_SPEED;

    obstacle = LOW;

    if (distance_mm < 150)
    {
      obstacle = HIGH;
    }

    state_det = LOW;
  }
}

void isr_fin_epreuve()
{
  state = LOW;
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

void synchro_esclave(int ticks_maitre, int val_pwm)
{
  if (abs(ticks_maitre - ticks1_0) > abs(ticks2 - ticks2_0))
  {
    analogWrite(pinPWM2, val_pwm);
  }
}


void avancer_Kp(float distance_cm)
{
  float delta = 0;
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


  if (distance_cm == 65.5) delta = delta_655;

  int distance_ticks = floor((distance_cm - delta) * 636.0 / (TWOPI * rayon)) ;
  ticks1_0 = ticks1;
  ticks2_0 = ticks2;

  while (abs(distance_ticks) - abs(ticks1 - ticks1_0) > 0)
  {
    if (state == LOW) break; // fin de l'épreuve
    test_det();
    if (obstacle == LOW)
    {
      digitalWrite(pinBRAKE1, LOW);
      digitalWrite(pinBRAKE2, LOW);

      int erreur = abs(distance_ticks) - abs(ticks1 - ticks1_0);

      if (Kp * erreur > 255)
      {
        analogWrite(pinPWM1, 255);
        synchro_esclave(ticks1, 255);
        Serial.println("IF SAT");

      }
      else
      {
        int consigne = Kp * erreur + PWM_min;
        analogWrite(pinPWM1, consigne);
        synchro_esclave(ticks1, consigne);
        Serial.println("IF NOT SAT");
      }

    }

    if (obstacle == HIGH)
    {
      Serial.println("IF NOT SAT 5");
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



void tourner_Kp(float angle)
{


  digitalWrite(pinBRAKE1, LOW);
  digitalWrite(pinBRAKE2, LOW);

  float distance_cm = abs(angle) * dist_roues / 2;
  int distance_ticks = floor(distance_cm / (TWOPI * rayon) * 636.0);
  ticks1_0 = ticks1;
  ticks2_0 = ticks2;

  if (angle < 0)
  {
    digitalWrite(pinDIR1, LOW); // à vérifier, il faut être sûr d'avancer et non pas de reculer!
    digitalWrite(pinDIR2, LOW);
    while (abs(distance_ticks) - abs(ticks1 - ticks1_0) > 0)
    {
      if (state == LOW) break; // fin de l'épreuve
      test_det();
      if (obstacle == LOW)
      {
        int erreur = abs(distance_ticks) - abs(ticks1 - ticks1_0);
        if (Kp * erreur > 255)
        {
          analogWrite(pinPWM1, 255);
          synchro_esclave(ticks1, 255);
        }
        else
        {
          analogWrite(pinPWM1, Kp * erreur + PWM_min);
          synchro_esclave(ticks1, Kp * erreur + PWM_min);
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
  }

  if (angle > 0)
  {

    digitalWrite(pinDIR1, HIGH);
    digitalWrite(pinDIR2, HIGH);
    while (abs(distance_ticks) - abs(ticks1 - ticks1_0) > 0)
    {
      if (state == LOW) break; // fin de l'épreuve
      test_det();
      if (obstacle == LOW)
      {
        int erreur = abs(distance_ticks) - abs(ticks1 - ticks1_0);
        if (Kp * erreur > 255)
        {
          analogWrite(pinPWM1, 255);
          synchro_esclave(ticks1, 255);
        }
        else
        {
          analogWrite(pinPWM1, Kp * erreur + PWM_min);
          synchro_esclave(ticks1, Kp * erreur + PWM_min);
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

  }
  analogWrite(pinPWM1, 0);
  analogWrite(pinPWM2, 0);
  digitalWrite(pinBRAKE1, HIGH);
  digitalWrite(pinBRAKE2, HIGH);
}

void avancer_TOR(void)
{
  while (TOR = LOW)
  {
    if (state == LOW) break; // fin de l'épreuve
    test_det_TOR();
    analogWrite(pinPWM1, 255);
    synchro_esclave(ticks1, 255);
  }
  analogWrite(pinPWM1, 0);
  analogWrite(pinPWM2, 0);
  digitalWrite(pinBRAKE1, HIGH);
  digitalWrite(pinBRAKE2, HIGH);
}

void salve_catapulte()
{
  int k;
  for (k = 0; k < 10; k++)
  {
    servo_catapulte.write(80); ///On fait tourner le servo
    delay(5000);
    servo_catapulte.write(100); // Pause avant la prochaine salve
    delay(500);
  }
}


