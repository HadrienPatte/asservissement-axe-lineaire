/* -------------------------------------------------------------------------------------------------------------------------
  Pilotage moteur DC avec 2 boutons et un potentiometre

  Deux ILS gèrent les fin de course (interrupteur NC, ferme, donc 0, car connecte à GND par defaut)
  Le homing se fait sur l'ILS gauche

  Implementation d'un asservissement PID par bibliothèque intégrée
  Implementation de la vitesse instantanee par integration de la position

  La course est de 17904 impulsions

  V 5  Hadrien Patte 25/02/2017
  ----------------------------------------------------------------------------------------------------------------------------*/
/* -------------------------------------------------------------------------------------------------------------------------
  Notes : * pas de pulldown internes sur des arduino, uniquement des pullup
            volatile pour les variables modifiables en interruption
            const pour les pin
  ----------------------------------------------------------------------------------------------------------------------------*/
#include <PID_v2.h>

double consigne = 8000;
volatile double position = 0;             // position mesuree par le codeur

long previousMillis = 0;                // pour l'affichage de la position sur le port serie


// Connexion du driver moteur L298N aux broches numeriques Arduino
const byte pinPWM             =  12;     // Pin PWM enable
const byte pinMoteur1         =  11;     // Pin Moteur 1
const byte pinMoteur2         =  10;     // Pin Moteur 2

const byte pinCodeurA         =   2;     // Broche signal codeur A
const byte pinCodeurB         =   3;     // Broche signal codeur B
boolean stateCodeurA          =   0;     // Variable d'etat du codeur A
boolean stateCodeurB          =   0;     // Variable d'etat du codeur B


const byte pinILSDroit        =  14;     // Broche signal de l'ILS droit
const byte pinILSGauche       =  15;     // Broche signal de l'ILS gauche (celui qui sert pour le homing)

int homingSpeed               = 200;     // Vitesse de homing
int vitesse                   = 255;


volatile bool sens = false;

float vitesseInstantanee = 0;

const int period = 50;

//PID
int periodPID = 100;
float kp = 1;
float ki = 0;
float kd = 0;
//float previousError = 0;
float previousPosition;
float sumError;
double output;
unsigned long previousTime;

PID myPID(&position, &output, &consigne, kp, ki, kd, DIRECT);

void setup() {
  Serial.begin(115200);           // Activation de la communication sur le port série

  // Initialisation des broches de commande du moteur
  pinMode(pinPWM,     OUTPUT);
  pinMode(pinMoteur1, OUTPUT);
  pinMode(pinMoteur2, OUTPUT);

  // Initialisation des broches des codeurs
  pinMode(pinCodeurA,  INPUT);
  pinMode(pinCodeurB,  INPUT);

  // Initialisation des broches des ILS
  pinMode(pinILSDroit,      INPUT);      // configure la broche de l'ILS droit en entrée
  pinMode(pinILSGauche,     INPUT);      // configure la broche de l'ILS gauche en entrée
  digitalWrite(pinILSDroit,  HIGH);      // Activation pullup interne
  digitalWrite(pinILSGauche, HIGH);      // Activation pullup interne

  // Initialise la position
  homing(homingSpeed);

  // Lance les interruptions
  attachInterrupt(0, interruptionCodeurA, CHANGE); // interruption 0 sur le codeur A (PIN 2)
  attachInterrupt(1, interruptionCodeurB, CHANGE); // interruption 1 sur le codeur B (PIN 3)

  // Initialise le PID
  myPID.SetMode(AUTOMATIC);
  myPID.SetSampleTime(2);
  myPID.SetOutputLimits(-255, 255);
}

void loop() {
  // boucle de controle asservi (reponse a une consigne)
  myPID.Compute();

  // Affichage sur le port série toutes les period
  if (millis() > previousMillis + period )  {
    Serial.println(position);
    previousMillis = millis();
  }

  /*// Test de fin de course
    if ((digitalRead(pinILSGauche) == 1) or (digitalRead(pinILSDroit) == 1)) {
    digitalWrite(pinMoteur1, LOW);            // Arret du moteur
    digitalWrite(pinMoteur2, LOW);
    }*/


  if (output > 200) {
    deplacementDroite(output);
  }
  else if (output < -200) {
    deplacementGauche(abs(output));
  }
  else {
    arretMoteur();
  }

}

// ---------------------------------------------------------------------------------------------------------------------------
void homing(int vitesseHoming) {
  // On Fait avancer le chariot vers la gauche tant que l'ILS gauche a le status 0. On arrete le moteur quand l'ILS passe au status 1
  analogWrite(pinPWM, homingSpeed);            // Envoi de la vitesse de homing sur la pin PWM
  while (digitalRead(pinILSGauche) == LOW) {
    deplacementGauche(vitesseHoming);
  }
  arretMoteur();
  delay(300);

  while (digitalRead(pinILSGauche) == HIGH) {
    deplacementDroite(vitesseHoming);
  }
  arretMoteur();
  delay(300);
}

// ---------------------------------------------------------------------------------------------------------------------------
void deplacementGauche(int vitesse) {
  // Déplacement vers la gauche du chariot à la vitesse vitesse
  analogWrite(pinPWM,   vitesse);            // Envoi de la vitesse sur la pin PWM pinPWM
  digitalWrite(pinMoteur1,  LOW);
  digitalWrite(pinMoteur2, HIGH);
}

// ---------------------------------------------------------------------------------------------------------------------------
void deplacementDroite(int vitesse) {
  // Déplacement vers la droite du chariot
  analogWrite(pinPWM,   vitesse);            // Envoi de la vitesse sur la pin PWM pinPWM
  digitalWrite(pinMoteur1, HIGH);
  digitalWrite(pinMoteur2,  LOW);
}

// ---------------------------------------------------------------------------------------------------------------------------
void arretMoteur() {
  // Arret du moteur
  analogWrite(pinPWM,        0);
  digitalWrite(pinMoteur1, LOW);
  digitalWrite(pinMoteur2, LOW);
}

// ---------------------------------------------------------------------------------------------------------------------------
void deplacementPosition(long consigne) {
  // deplacement jusqu a la position consigne avec retroaction
  while (abs(consigne - position) > 10) {
    if (consigne > position) {
      deplacementDroite(vitesse);
    }
    else {
      deplacementGauche(vitesse);
    }
  }
  arretMoteur();
}

// ---------------------------------------------------------------------------------------------------------------------------

// Code for interrupt 0. Gives the encoder signed cumulated ticks, including CW and CCW (plus and minus) ways

void interruptionCodeurA() {
  //stateCodeurA = (PINE & B00010000) >> 4;
  //stateCodeurB = (PINH & B00100000) >> 5;   //que pour la pin 8

  stateCodeurA = digitalRead(pinCodeurA);
  stateCodeurB = digitalRead(pinCodeurB);
  if (stateCodeurA == HIGH) {              // Found a low-to-high on A phase. if(digitalRead(pinCodeurA)==HIGH){ .... read PE4
    if (stateCodeurB == LOW) {             // Check B phase to see which way. if(digitalRead(pinCodeurB)==LOW) { .... read PH5
      position -- ;           // CCW
      sens = false;
    }
    else {
      position ++ ;                      // CW
      //sens = true;
    }
  }
  else {
    if (stateCodeurB == LOW) {             // Check B phase to see which way. if(digitalRead(pinCodeurB)==LOW) { .... read PH5
      position ++ ;                       // CCW
      sens = true;
    }
    else {
      position -- ;                       // CW
      //sens = false;
    }
  }
}
// ---------------------------------------------------------------------------------------------------------------------------

void interruptionCodeurB() {
  //stateCodeurA = (PINE & B00010000) >> 4;
  //stateCodeurB = (PINH & B00100000) >> 5;   //que pour la pin 8

  stateCodeurA = digitalRead(pinCodeurA);
  stateCodeurB = digitalRead(pinCodeurB);
  if (stateCodeurA == HIGH) {              // Found a low-to-high on A phase. if(digitalRead(pinCodeurA)==HIGH){ .... read PE4
    if (stateCodeurB == LOW) {             // Check B phase to see which way. if(digitalRead(pinCodeurB)==LOW) { .... read PH5
      position ++ ;           // CCW
      sens = false;
    }
    else {
      position -- ;                      // CW
      //sens = true;
    }
  }
  else {
    if (stateCodeurB == LOW) {             // Check B phase to see which way. if(digitalRead(pinCodeurB)==LOW) { .... read PH5
      position -- ;                       // CCW
      sens = true;
    }
    else {
      position ++ ;                       // CW
      //sens = false;
    }
  }
}
// ---------------------------------------------------------------------------------------------------------------------------

// ---------------------------------------------------------------------------------------------------------------------------
void correcteur() {
  // Correcteur PID
  unsigned long now = millis();
  if ((now - previousTime) >= periodPID) {
    // on fait les calculs
    float error = consigne - position;
    sumError += error;
    float dPosition = position - previousPosition;
    // on calcule output
    output = kp * error + ki * sumError - kd * dPosition;

    // pour le prochain passage
    previousPosition = position;
    previousTime = now;
  }
}
