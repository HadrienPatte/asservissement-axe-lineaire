/* -------------------------------------------------------------------------------------------------------------------------
  Pilotage moteur DC asservi

  Deux ILS gerent les fin de course (interrupteur NC, ferme, donc 0, car connecte a GND par defaut)
  Le homing se fait sur l'ILS gauche

  Asservissement PID implemente intrinsequement precis

  La course est de 17904 impulsions

  V 7  Hadrien Patte 13/03/2017
  ----------------------------------------------------------------------------------------------------------------------------*/
/* -------------------------------------------------------------------------------------------------------------------------
  Notes : * pas de pulldown internes sur des arduino, uniquement des pullup
            volatile pour les variables modifiables en interruption
            const pour les pin
  ----------------------------------------------------------------------------------------------------------------------------*/
// Variables liees a l'affichage sr le port serie
unsigned long maintenant;
unsigned long dernierAffichage;
const int periodeAffichage = 50;

// Variables liees au calcul du PID
unsigned long dernierCalculPID = 0;
volatile float position = 0; // input
float dernierePosition = 0;
float consigne = 10000; // setpoint
float sortiePID = 0; // output
float ITerm = 0;
int periodePID = 10;
int vitesseMin = 200;
int vitesseMax = 255;

// Coefficients PID
float kp = 1;
float ki = 10;
float kd = 0.5;
/*float kp = 1;
  float ki = 0;
  float kd = 0;*/

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

int homingSpeed               = 200;     // Vitesse de homing (0-255)

void setup() {
  Serial.begin(115200);                  // Activation du moniteur serie

  // Initialisation des broches de commande du moteur
  pinMode(pinPWM,     OUTPUT);
  pinMode(pinMoteur1, OUTPUT);
  pinMode(pinMoteur2, OUTPUT);

  // Initialisation des broches des codeurs
  pinMode(pinCodeurA,  INPUT);
  pinMode(pinCodeurB,  INPUT);

  // Initialisation des broches des ILS
  pinMode(pinILSDroit,      INPUT);      // configure la broche de l'ILS droit en entree
  pinMode(pinILSGauche,     INPUT);      // configure la broche de l'ILS gauche en entree
  digitalWrite(pinILSDroit,  HIGH);      // Activation pullup interne
  digitalWrite(pinILSGauche, HIGH);      // Activation pullup interne

  // Initialise la position
  homing(homingSpeed);

  // Lance les interruptions
  attachInterrupt(0, interruptionCodeurA, CHANGE); // interruption 0 sur le codeur A (PIN 2)
  attachInterrupt(1, interruptionCodeurB, CHANGE); // interruption 1 sur le codeur B (PIN 3)

  // Initialise le PID
  ki = ki * ((float)periodePID) / 1000;
  kd = kd * 1000 / ((float)periodePID);
}

void loop() {
  // boucle de controle asservi (reponse a une consigne)
  afficher(position);

  calculPID();

  /*// Test de fin de course
    if ((digitalRead(pinILSGauche) == 1) or (digitalRead(pinILSDroit) == 1)) {
    digitalWrite(pinMoteur1, LOW);            // Arret du moteur
    digitalWrite(pinMoteur2, LOW);
    }*/


  if ( (consigne - position) > 5) {
    deplacementDroite(sortiePID);
  }
  else if ( (consigne - position) < -5) {
    deplacementGauche(abs(sortiePID));
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
  // Deplacement vers la gauche du chariot a la vitesse vitesse
  analogWrite(pinPWM,   vitesse);            // Envoi de la vitesse sur la pin PWM pinPWM
  digitalWrite(pinMoteur1,  LOW);
  digitalWrite(pinMoteur2, HIGH);
}

// ---------------------------------------------------------------------------------------------------------------------------
void deplacementDroite(int vitesse) {
  // Deplacement vers la droite du chariot
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
void afficher(volatile double variable) {
  // Affichage sur le port serie toutes les period de la variable
  maintenant = millis();
  if ( (maintenant - dernierAffichage) >= periodeAffichage )  {
    //if (abs(variable - consigne) > 5) {
    Serial.println(variable);
    //}
    dernierAffichage = maintenant;
  }
}

// ---------------------------------------------------------------------------------------------------------------------------
void calculPID()
{
  maintenant = millis();
  if ( (maintenant - dernierCalculPID) >= periodePID)
  {
    //Compute all the working erreur variables
    float erreur = (consigne - position);
    ITerm += (ki * erreur);
    if (ITerm > vitesseMax) {
      ITerm = vitesseMax;
    }
    else if (ITerm < vitesseMin) {
      ITerm = vitesseMin;
    }
    float dposition = (position - dernierePosition);

    // Calcul de sortiePID
    sortiePID = kp * erreur + ITerm - kd * dposition;

    // On cale sortiePID entre vitesseMin et vitesseMAX
    if (sortiePID >= 0) {
      if (sortiePID > vitesseMax) {
        sortiePID = vitesseMax;
      }
      else if (sortiePID < vitesseMin) {
        sortiePID = vitesseMin;
      }
    }
    else {
      if (sortiePID > (0 - vitesseMin) ) {
        sortiePID = (0 - vitesseMin);
      }
      else if (sortiePID < (0 - vitesseMax)) {
        sortiePID = (0 - vitesseMax);
      }
    }

    // Pour le prochain passage
    dernierePosition = position;
    dernierCalculPID = maintenant;
  }
}

// ---------------------------------------------------------------------------------------------------------------------------
void interruptionCodeurA() {
  //stateCodeurA = (PINE & B00010000) >> 4;
  //stateCodeurB = (PINH & B00100000) >> 5;   //que pour la pin 8

  stateCodeurA = digitalRead(pinCodeurA);
  stateCodeurB = digitalRead(pinCodeurB);
  if (stateCodeurA == HIGH) {              // Found a low-to-high on A phase. if(digitalRead(pinCodeurA)==HIGH){ .... read PE4
    if (stateCodeurB == LOW) {             // Check B phase to see which way. if(digitalRead(pinCodeurB)==LOW) { .... read PH5
      position -- ;
    }
    else {
      position ++ ;
    }
  }
  else {
    if (stateCodeurB == LOW) {             // Check B phase to see which way. if(digitalRead(pinCodeurB)==LOW) { .... read PH5
      position ++ ;
    }
    else {
      position -- ;
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
      position ++ ;
    }
    else {
      position -- ;
    }
  }
  else {
    if (stateCodeurB == LOW) {             // Check B phase to see which way. if(digitalRead(pinCodeurB)==LOW) { .... read PH5
      position -- ;
    }
    else {
      position ++ ;
    }
  }
}

// ---------------------------------------------------------------------------------------------------------------------------
