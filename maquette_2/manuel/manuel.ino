/* -------------------------------------------------------------------------------------------------------------------------
  Pilotage moteur DC avec 2 boutons et un potentiometre

  Le bouton de gauche (rouge) fait tourner le moteur dans un sens, l'autre bouton (vert) dans l'autre sens
  Le potentiomètre permet de régler la vitesse
  Deux ILS gèrent les fin de course (interrupteur NC, ferme, donc 0, car connecte à GND par defaut)
  Le homing se fait sur l'ILS gauche


  Test en cours des interruptions pour récpérer les informations des codeurs du moteur

  ----------------------------------------------------------------------------------------------------------------------------*/


// Connexion du driver moteur L298N aux broches numeriques Arduino
const int enA          =  9;      // Pin PWM enable
const int in1          =  10;     // Pin Moteur 1
const int in2          =  11;     // Pin Moteur 2

const int encoderPinA  =   2;     // Broche signal codeur A
const int encoderPinB  =   3;     // Broche signal codeur B
const int rougePin     =   5;     // Broche signal bouton rouge
const int vertPin      =   6;     // Broche signal bouton vert
int rougeState         =   1;     // Variable de status du bouton rouge
int vertState          =   1;     // Variable de status du bouton vert

int testPin = 13;
int vitesse            =   0;     // Vitesse lue sur le potentiometre

int stateA = 0;
int stateB = 0;
void setup() {


  pinMode(enA,      OUTPUT);
  pinMode(in1,      OUTPUT);
  pinMode(in2,      OUTPUT);

  pinMode(encoderPinA,  INPUT);
  pinMode(encoderPinB,  INPUT);

  pinMode(rougePin, INPUT);
  pinMode(vertPin,  INPUT);
  digitalWrite(rougePin, HIGH);  // Activation pullup interne
  digitalWrite(vertPin,  HIGH);  // Activation pullup interne
  pinMode(testPin, OUTPUT);

}

void loop() {
stateA = digitalRead(encoderPinA);
stateB = digitalRead(encoderPinB);
  if (stateB == HIGH) {
    digitalWrite(testPin, HIGH);
  }
  else {
    digitalWrite(testPin, LOW);
  }
  rougeState = !digitalRead(rougePin);   // On inverse la lecture car on veut le moteur arrete quand la pin est à l'état haut
  vertState  = !digitalRead(vertPin);    // On inverse la lecture car on veut le moteur arrete quand la pin est à l'état haut

  vitesse = 255;
  analogWrite(enA, vitesse);                           // Envoi de la vitesse sur la pin PWM enA

  // Test des boutons
  if (vertState == HIGH) {
    deplacementDroite();
  }
  else if (rougeState == HIGH) {
    deplacementGauche();
  }
  else {
    arretMoteur();
  }
}


// ---------------------------------------------------------------------------------------------------------------------------
void deplacementGauche() {
  // Déplacement vers la gauche du chariot
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
}

// ---------------------------------------------------------------------------------------------------------------------------
void deplacementDroite() {
  // Déplacement vers la droite du chariot
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
}

// ---------------------------------------------------------------------------------------------------------------------------
void arretMoteur() {
  // Arret du moteur
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
}

