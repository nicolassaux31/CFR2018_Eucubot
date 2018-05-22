// DEFINITION DES CONSTANTES

#include <DueTimer.h> // Bibliothèque modifiée (pour compatibilité avec Servo): Timer0, 2,3,4 and 5 ne sont pas utilisables
#include <Servo.h>
#define TWOPI 6.283185307179586476925286766559 //2*PI
float rayon = 3.481; // rayon de la roue en cm - A VERIFIER!
float dist_roues = 26.1; // distance entre les roues en cm - A VERIFIER!
volatile long int ticks1 = 0; // Position du moteur en "ticks" (sachant qu'un tour contient 636 ticks). It will increment or decrement each time there is a change in the sensors
volatile long int ticks2 = 0; // Pulse counter variable. It will increment or decrement each time there is a change in the sensors
int ticks1_0; // Position de référence lors de l'appel de tourner() / avancer()
int ticks2_0;
float Kp = 0.7; // Gain du correcteur proportionnel
int PWMmin = 80; // Prise en compte du seuil des moteurs
volatile byte state_det = LOW; // bit passant à HIGH lorsque le timer lié au capteur IR déclenche l'interruption
volatile byte obstacle = LOW; // bit passant à HIGH lorsqu'un obstacle est détecté par le capteur IR associé au robot adverse
const int pinLED = 23; // Définition pin pour la LED
const int pinCouleur = 25;
bool couleur_vert;
const int pinJack = 27;
volatile byte epreuve_en_cours = HIGH; //bit passant à "LOW" quand l'épreuve est terminée et impliquant des break à l'entrée des fonctions de déplacement
const int sharp = 7; // pin associé au capteur IR détectant le robot adverse
int delta_esclave = 0; // delta que l'on sourstrait à la PWM du moteur esclave pour garantir que le robot roule droit #truand
const int pinPWMcata = 7;
const int pinTOR1 = 51;
const int pinTOR2 = 47;
volatile byte timeout = LOW;

// DEFINITION DES MOTEURS

//Jeton de lancement timer
int start_timer=1; //Jeton pour lancement timer quand jack retiré
int duree_epreuve_en_sec = 100;


/*
  const int pinDIR1 = 13; //choix du sens de rotation du moteur 1
  const int pinPWM1 = 11;
  const int pinBRAKE1 = 8;
  const int pinSensorA1 = 34; //bleu  (5)
  const int pinSensorB1 = 32; //violet  (6)

  const int pinDIR2 = 12; //choix du sens de rotation du moteur 2 (HIGH / LOW)
  const int pinPWM2 = 3; //choix de la vitesse de rotation (0 à 255)
  const int pinBRAKE2 = 9; //pin sur HIGH pour activer les freins (HIGH / LOW)
  const int pinSensorA2 = 50; //bleu  (2)
  const int pinSensorB2 = 52; //violet   (4)
*/

const int pinDIR2 = 13; //choix du sens de rotation du moteur 1 MOTEUR GAUCHE
const int pinPWM2 = 11;
const int pinBRAKE2 = 8;
const int pinSensorA2 = 34; //bleu  (5)
const int pinSensorB2 = 32; //violet  (6)

const int pinDIR1 = 12; //choix du sens de rotation du moteur 2 (HIGH / LOW) MOTEUR DROIT
const int pinPWM1 = 3; //choix de la vitesse de rotation (0 à 255)
const int pinBRAKE1 = 9; //pin sur HIGH pour activer les freins (HIGH / LOW)
const int pinSensorA1 = 50; //bleu  (2)
const int pinSensorB1 = 52; //violet   (4)

// DEFINITION DES SERVOMOTEURS

Servo servo_catapulte;
Servo servo_bras_droit;
Servo servo_bras_gauche;
Servo servo_bras_abeille;

void setup() {

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
  pinMode(pinPWMcata, OUTPUT);

  servo_catapulte.attach(10);
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


  servo_bras_abeille.write(80); // poisition basse : 0 / position haute : 80

} // fin setup

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
    if(start_timer==1) //on passe une fois au debut de l'épreuve
    {
          Timer6.attachInterrupt(isr_fin_epreuve).start(duree_epreuve_en_sec * 1000000); // fin de l'épreuve (à 100sec)
          start_timer=0;
    }

    if (couleur_vert) // On entre dans la boucle VERTE
    {
      digitalWrite(pinLED, HIGH); // On allume la LED
      
      //on avance jusqu'à l'interrupteur
      avancer_Kp(85);
      delay(2000);
      //on tourne vers l'interrupteur
      float ang_deg = 65;
      float ang_rad = ang_deg * TWOPI / 360;
      obstacle = LOW;
      tourner_Kp(ang_rad);
      delay(2000);
      //on avance vers l'interrupteur jusqu'à l'activer
      if(epreuve_en_cours){
      obstacle = LOW;
      digitalWrite(pinBRAKE1, LOW);
      digitalWrite(pinBRAKE2, LOW);
      digitalWrite(pinDIR1, LOW);
      digitalWrite(pinDIR2, HIGH);
      analogWrite(pinPWM1, 255);
      analogWrite(pinPWM2, 255);
      delay(1000);
      }
      //on s'arrête contre l'interrupteur
      analogWrite(pinPWM1, 0);
      analogWrite(pinPWM2, 0);
      digitalWrite(pinBRAKE1, LOW);
      digitalWrite(pinBRAKE2, LOW);
      delay(1000);
      //on fait demi tour
      ang_deg = 170;
      ang_rad = ang_deg * TWOPI / 360;
      obstacle = LOW;
      tourner_Kp(ang_rad);
      //on avance vers l'épuration
      analogWrite(pinPWM1, 0);
      analogWrite(pinPWM2, 0);
      digitalWrite(pinBRAKE1, LOW);
      digitalWrite(pinBRAKE2, LOW);
      digitalWrite(pinDIR1, LOW);
      digitalWrite(pinDIR2, HIGH);
      obstacle = LOW;
      avancer_Kp(85);
      delay(1000);
      //on tourne vers les blocs
      ang_deg = -80; //SIGNE MOINS
      ang_rad = ang_deg * TWOPI / 360;
      tourner_Kp(ang_rad);
      delay(1000);
      //on avance un peu
      avancer_Kp(65);
      delay(1000);
      //on tourne deux fois pour petit angle
      ang_deg = 35; //SIGNE MOINS
      ang_rad = ang_deg * TWOPI / 360;
      tourner_Kp(ang_rad);
      delay(1000);
      avancer_Kp(20);
      delay(1000);
      ang_deg = -55; //SIGNE MOINS
      ang_rad = ang_deg * TWOPI / 360;
      tourner_Kp(ang_rad);
      delay(1000);
      //on avance jusqu'à se caler contre le mur
      if(epreuve_en_cours){
      digitalWrite(pinBRAKE1, LOW);
      digitalWrite(pinBRAKE2, LOW);
      digitalWrite(pinDIR1, LOW);
      digitalWrite(pinDIR2, HIGH);
      avancer_TOR();
      obstacle = LOW;
      //delay(1000);
      //avancer_Kp(40);
      delay(1000);
      }
      //on tourne vers l'abeille
      ang_deg = 85; //SIGNE PLUS 85
      ang_rad = ang_deg * TWOPI / 360;
      tourner_Kp(ang_rad);
      delay(1000);
      //on sort le bras
      if(epreuve_en_cours){
      servo_bras_abeille.write(20);
      }
      delay(1000);
      //on avance jusqu'au mur
      if(epreuve_en_cours){
      digitalWrite(pinBRAKE1, LOW);
      digitalWrite(pinBRAKE2, LOW);
      digitalWrite(pinDIR1, LOW);
      digitalWrite(pinDIR2, HIGH);
      avancer_TOR();
      delay(1000);
      }
      //on tourne pour la pousser
      ang_deg = 770; //SIGNE PLUS 60
      ang_rad = ang_deg * TWOPI / 360;
      tourner_Kp(ang_rad);
      delay(500);
      ang_deg = 20; //SIGNE PLUS 60
      ang_rad = ang_deg * TWOPI / 360;
      tourner_Kp(ang_rad);
      delay(500);
      servo_bras_abeille.write(80);
      delay(500);
      avancer_Kp(55); // avancer après avoir descendu l'abeille
      delay(1000);
      ang_deg = 30;
      ang_rad = ang_deg * TWOPI / 360;
      tourner_Kp(ang_rad);// tourner pour suivre ligne du milieu
      delay(1000);
      avancer_Kp(190); // avancer pour aller plus loin que les balles
      delay(1000);
      avancer_Kp(-5);
      delay(1000);
      ang_deg = -45;
      ang_rad = ang_deg * TWOPI / 360;
      tourner_Kp(ang_rad);
      delay(1000);
      delay(1000000);
      
    }
    else // On entre dans la boucle ORANGE
    {

      digitalWrite(pinLED, LOW); // On éteint la LED
      //on avance jusqu'à l'abeille
      avancer_Kp(80);
      delay(2000);
      //on tourne vers l'interrupteur
      Serial.println(timeout);
      float ang_deg = -90;
      float ang_rad = ang_deg * TWOPI / 360;
      obstacle = LOW;
      tourner_Kp(ang_rad);
      Serial.println(timeout);
      delay(2000);
      //on avance jusqu'à l'interrupteur
      if(epreuve_en_cours){
      obstacle = LOW;
      digitalWrite(pinBRAKE1, LOW);
      digitalWrite(pinBRAKE2, LOW);
      digitalWrite(pinDIR1, LOW);
      digitalWrite(pinDIR2, HIGH);
      analogWrite(pinPWM1, 255);
      analogWrite(pinPWM2, 255);
      delay(1000);
      }
      //on s'arrête à l'interrupteur
      analogWrite(pinPWM1, 0);
      analogWrite(pinPWM2, 0);
      digitalWrite(pinBRAKE1, LOW);
      digitalWrite(pinBRAKE2, LOW);
      delay(1000);
      //on fait demi tour
      timeout = LOW;
      Serial.println(timeout);
      ang_deg = 160;
      ang_rad = ang_deg * TWOPI / 360;
      obstacle = LOW;
      tourner_Kp(ang_rad);
      //on avance vers l'abeille
      analogWrite(pinPWM1, 0);
      analogWrite(pinPWM2, 0);
      digitalWrite(pinBRAKE1, LOW);
      digitalWrite(pinBRAKE2, LOW);
      digitalWrite(pinDIR1, LOW);
      digitalWrite(pinDIR2, HIGH);
      obstacle = LOW;
      avancer_Kp(90);//80
      delay(1000);
      //on tourne une seconde fois
      ang_deg = 45;
      ang_rad = ang_deg * TWOPI / 360;
      tourner_Kp(ang_rad);
      delay(1000);
      //on avance une deuxième fois
      avancer_Kp(90);
      delay(1000);
      //on avance jusqu'à se caler contre le mur
      if(epreuve_en_cours){
      digitalWrite(pinBRAKE1, LOW);
      digitalWrite(pinBRAKE2, LOW);
      digitalWrite(pinDIR1, LOW);
      digitalWrite(pinDIR2, HIGH);
      avancer_TOR();
      delay(1000);
      }
      //on tourne vers l'abeille
      ang_deg = -68;
      ang_rad = ang_deg * TWOPI / 360;
      tourner_Kp(ang_rad);
      delay(1000);
      //on sort le bras
      if(epreuve_en_cours){
      servo_bras_abeille.write(20);
      delay(1000);
      }
      //on avance jusqu'au mur
      if(epreuve_en_cours){
      digitalWrite(pinBRAKE1, LOW);
      digitalWrite(pinBRAKE2, LOW);
      digitalWrite(pinDIR1, LOW);
      digitalWrite(pinDIR2, HIGH);
      avancer_TOR();
      delay(1000);
      }
      //on tourne pour la pousser
      ang_deg = -720;
      ang_rad = ang_deg * TWOPI / 360;
      tourner_Kp(ang_rad);
      delay(500);
      ang_deg = -20;
      ang_rad = ang_deg * TWOPI / 360;
      tourner_Kp(ang_rad);
      delay(500);
      servo_bras_abeille.write(80);
      delay(500);
      avancer_Kp(68); // avancer après avoir descendu l'abeille
      delay(1000);
      ang_deg = -73;
      ang_rad = ang_deg * TWOPI / 360;
      tourner_Kp(ang_rad);// tourner pour suivre ligne du milieu
      delay(1000);
      avancer_Kp(190); // avancer pour aller plus loin que les balles
      delay(1000);
      delay(1000000);
      
    }
  }

} //fin loop

// SYNCHRONISATION DU MOTEUR MAîTRE AVEC LE MOTEUR ESCLAVE

void synchro_esclave(int ticks_maitre, int val_pwm)
{
  if (abs(ticks_maitre - ticks1_0) > abs(ticks2 - ticks2_0)) // on compare la position du moteur esclave à celle du moteur maître
    // si le robot esclave est en retard, on l'actionne
  {
    analogWrite(pinPWM2, val_pwm);
  }
}

// PRISE EN COMPTE DE LA DUREE DE L'EPREUVE

void isr_fin_epreuve()
{
  epreuve_en_cours = LOW;
}

// MISE A JOUR DE LA POSITION DE CHAQUE MOTEUR

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

// TEST DE DETECTION

void test_det(void) { // fonction de détection, elle scrute le capteur IR pour vérifier si le robot adverse n'est pas détecté (elle est appelée dans avancer_Kp)
  if (state_det == HIGH)
  {

    float voltage_sharp = analogRead(sharp);

    // CAPTEUR IR 1
    float distance = -3.0 * pow(10, -7) * pow(voltage_sharp, 3) + 0.0006 * pow(voltage_sharp, 2) - 0.398 * voltage_sharp + 89.455; // distance(mm) = f(voltage_sharp)

    // CAPTEUR IR 2
    //float distance = -1*pow(10,-7)*pow(voltage_sharp,3) + 0.0003*pow(voltage_sharp,2) - 0.2612*voltage_sharp + 82.484;

    //Serial.println("distance");
    //Serial.println(distance);

    obstacle = LOW;


    if (distance < 45)
    {
      obstacle = HIGH;
    }

    //Serial.println("obstacle");
    //Serial.println(obstacle);

    state_det = LOW;
  }
}

// AVANCER AVEC CORRECTION PROPORTIONNELLE

void avancer_Kp(float distance_cm) // fait avancer le robot tout en scrutant la présence du robot adverse
{
  timeout = LOW;
  Timer6.attachInterrupt(isr_timeout).start(10000000);
  digitalWrite(pinBRAKE1, LOW);
  digitalWrite(pinBRAKE2, LOW);

  if (distance_cm > 0)
  {
    digitalWrite(pinDIR1, LOW); // à vérifier, il faut être sûr d'avancer et non pas de reculer!
    digitalWrite(pinDIR2, HIGH);
  }

  else
  {
    digitalWrite(pinDIR1, HIGH); // à vérifier, il faut être sûr d'avancer et non pas de reculer!
    digitalWrite(pinDIR2, LOW);
  }

  int distance_ticks = floor((distance_cm) * 636.0 / (TWOPI * rayon)) ; // on calcule la distance totale à parcourir
  ticks1_0 = ticks1; // on note la position d'origine pour calculer l'avancée du trajet
  ticks2_0 = ticks2;

  while (abs(distance_ticks) - abs(ticks1 - ticks1_0) > 0)
  {
    if (timeout) break;
    if (epreuve_en_cours == LOW) break; // fin de l'épreuve
    test_det(); // on scrute ici le capteur IR
    if (obstacle == LOW)
    {
      digitalWrite(pinBRAKE1, LOW);
      digitalWrite(pinBRAKE2, LOW);

      int erreur = abs(distance_ticks) - abs(ticks1 - ticks1_0);

      //Serial.println("erreur");
      //Serial.println(erreur);

      if (Kp * erreur > 255) // cas d'actionneur saturé (PWM > 255)
      {
        analogWrite(pinPWM1, 255);
        analogWrite(pinPWM2, 255);
        //synchro_esclave(ticks1, 255 - delta_esclave);

      }
      else // cas d'actionneur non saturé (0 < PWM < 255)
      {

        int consigne = Kp * erreur + PWMmin;
        analogWrite(pinPWM1, consigne);
        analogWrite(pinPWM2, consigne);
        //synchro_esclave(ticks1, consigne - delta_esclave);

        //analogWrite(pinPWM1, 255);
        //analogWrite(pinPWM2, 255);
      }
      
    }

    if (obstacle == HIGH) // si un robot adverse est détecté, on s'arrête et on actionne les freins
    {
      analogWrite(pinPWM1, 0);
      analogWrite(pinPWM2, 0);
      digitalWrite(pinBRAKE1, HIGH);
      digitalWrite(pinBRAKE2, HIGH);
    }

  }
  timeout = LOW;
  analogWrite(pinPWM1, 0); // une fois la distance voulue parcourue, on s'arrête et on actionne les freins
  analogWrite(pinPWM2, 0);
  digitalWrite(pinBRAKE1, HIGH);
  digitalWrite(pinBRAKE2, HIGH);
}

void isr_US_robot()
{
  state_det = HIGH;
}
// ISR TIMEOUT

void isr_timeout(void)
{
  timeout = HIGH;
}

// TOURNER AVEC UNE CORRECTION PROPORTIONNELLE

void tourner_Kp(float angle)
{
  timeout = LOW;
  Timer6.attachInterrupt(isr_timeout).start(3000000);
  
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
    while ((abs(distance_ticks) - abs(ticks1 - ticks1_0) > 0))
    {
      if (timeout) break;
      if (epreuve_en_cours == LOW) break; // fin de l'épreuve
      //test_det();
      if (obstacle == LOW)
      {
        int erreur = abs(distance_ticks) - abs(ticks1 - ticks1_0);
        if (Kp * erreur > 255)
        {
          analogWrite(pinPWM1, 255);
          //analogWrite(pinPWM2, 255 - delta_esclave);
          synchro_esclave(ticks1, 255 - delta_esclave);
        }
        else
        {
          analogWrite(pinPWM1, Kp * erreur + PWMmin);
          analogWrite(pinPWM2, Kp * erreur + PWMmin - delta_esclave);
          //synchro_esclave(ticks1, Kp * erreur + PWMmin - delta_esclave);
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
    timeout = LOW;
  }

  if (angle > 0)
  {

    digitalWrite(pinDIR1, HIGH);
    digitalWrite(pinDIR2, HIGH);
    while ((abs(distance_ticks) - abs(ticks1 - ticks1_0) > 0))
    {
      if (timeout) break;
      Serial.println(timeout);
      if (epreuve_en_cours == LOW) break; // fin de l'épreuve
      //test_det();
      if (obstacle == LOW)
      {
        int erreur = abs(distance_ticks) - abs(ticks1 - ticks1_0);
        if (Kp * erreur > 255)
        {
          analogWrite(pinPWM1, 255);
          analogWrite(pinPWM2, 255 - delta_esclave);
          //synchro_esclave(ticks1, 255 - delta-esclave);
        }
        else
        {
          analogWrite(pinPWM1, Kp * erreur + PWMmin);
          analogWrite(pinPWM2, Kp * erreur + PWMmin - delta_esclave);
          //synchro_esclave(ticks1, Kp * erreur + PWMmin - delta_esclave);
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
    timeout = LOW;
  }
  analogWrite(pinPWM1, 0);
  analogWrite(pinPWM2, 0);
  digitalWrite(pinBRAKE1, HIGH);
  digitalWrite(pinBRAKE2, HIGH);
}

// AVANCER JUSQU'A LA DETECTION D'UN MUR

void avancer_mur(void)
{
  digitalWrite(pinDIR1, HIGH);
  digitalWrite(pinDIR2, LOW);
  volatile byte obstacle_mur = LOW;
  while (obstacle_mur == LOW)
  {
    if (epreuve_en_cours == LOW) break;

    float voltage_sharp = analogRead(sharp);

    // CAPTEUR IR 1
    float distance_mur = -3.0 * pow(10, -7) * pow(voltage_sharp, 3) + 0.0006 * pow(voltage_sharp, 2) - 0.398 * voltage_sharp + 89.455;

    Serial.println(distance_mur);

    if (distance_mur < 35 && distance_mur > 8)
    {
      obstacle_mur = HIGH;
    }

    digitalWrite(pinBRAKE1, LOW);
    digitalWrite(pinBRAKE2, LOW);
    analogWrite(pinPWM1, 255);
    synchro_esclave(ticks1, 255 - delta_esclave);
  }
  analogWrite(pinPWM1, 0);
  analogWrite(pinPWM2, 0);
  digitalWrite(pinBRAKE1, HIGH);
  digitalWrite(pinBRAKE2, HIGH);
  obstacle_mur = LOW;
}

void avancer_TOR(void)
{
  timeout = LOW;
  Timer6.attachInterrupt(isr_timeout).start(3000000);
  
  digitalWrite(pinDIR1, LOW);
  digitalWrite(pinDIR2, HIGH);
  int mur1 = digitalRead(pinTOR1);
  int mur2 = digitalRead(pinTOR2);

  while (((mur1, mur2) != (HIGH, HIGH)))
  {
    if(timeout) break;
    mur1 = digitalRead(pinTOR1);
    mur2 = digitalRead(pinTOR2);
    Serial.println(mur1);
    Serial.println(mur2);
    analogWrite(pinPWM1, 255);
    analogWrite(pinPWM2, 255);
  }
  timeout = LOW;
  analogWrite(pinPWM1, 0);
  analogWrite(pinPWM2, 0);
}


// http://arduino.blaisepascal.fr/index.php/2016/02/06/lisser-un-signal-analogique/
// BIBLIO SHARP
