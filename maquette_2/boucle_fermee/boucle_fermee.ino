/* -------------------------------------------------------------------------------------------------------------------------
  Pilotage moteur DC avec 2 boutons et lecture position

  Le bouton de gauche (rouge) fait tourner le moteur dans un sens, l'autre bouton (vert) dans l'autre sens

  Un interrupteur gere une fin de course et l'initialisation au homing (interrupteur NC, ferme, donc 0, car connecte à GND par defaut)

  Les codeurs sont lus en binaire (lecture rapide) et sur les fronts montants ET descendants des deux codeurs ( 2 interruptions )

  Implementation d'un asservissement PID par bibliotheque intégrée

  Le systeme physique est tres amorti, ce qui semble rendre une reponse oscillante difficile a mettre en place

  La course est de 6647 impulsions

  V 5  Hadrien Patte 25/02/2017
  ----------------------------------------------------------------------------------------------------------------------------*/
/* -------------------------------------------------------------------------------------------------------------------------
  Notes : * pas de pulldown internes sur des arduino, uniquement des pullup
            volatile pour les variables modifiables en interruption
            const pour les pin
  ----------------------------------------------------------------------------------------------------------------------------*/
#include <PID_v2.h>

double consigne = 3000;
volatile double position = 0;             // position mesuree par le codeur
long previousposition = 0;

// Variables liées à l'affichage sr le port série
unsigned long maintenant;
unsigned long dernierAffichage;
const int periodeAffichage = 50;

// Connexion du driver moteur L298N aux broches numeriques Arduino
const byte pinPWM             =  9;      // Pin PWM enable
const byte pinMoteur1         =  10;     // Pin Moteur 1
const byte pinMoteur2         =  11;     // Pin Moteur 2

const byte pinCodeurA         =   2;     // Broche signal codeur A
const byte pinCodeurB         =   3;     // Broche signal codeur B
boolean stateCodeurA          =   0;     // Variable d'etat du codeur A
boolean stateCodeurB          =   0;     // Variable d'etat du codeur B

const byte pinBoutonRouge     =   5;     // Broche signal bouton rouge
const byte pinBoutonVert      =   6;     // Broche signal bouton vert
boolean stateBoutonRouge      =   1;     // Variable d'etat du bouton rouge
boolean stateBoutonVert       =   1;     // Variable d'etat du bouton vert

const byte pinInterrupteur    =  12;     // Broche de l'interrupteur de fin de course et d'initialisation de la position

int vitesse                   = 255;     // Vitesse PWM du moteur (map 0-255)
int homingSpeed               = 255;     // Vitesse de homing


//PID
float kp = 2;
float ki = 0.11;
float kd = 0;
double output;

PID myPID(&position, &output, &consigne, kp, ki, kd, DIRECT);

void setup() {
  Serial.begin(115200);                 // Activation du moniteur serie

  // Initialisation des broches de commande du moteur
  pinMode(pinPWM,     OUTPUT);
  pinMode(pinMoteur1, OUTPUT);
  pinMode(pinMoteur2, OUTPUT);

  // Initialisation des broches des codeurs
  pinMode(pinCodeurA, INPUT);
  pinMode(pinCodeurB, INPUT);

  // Initialisation de la broche de l'interrupteur
  pinMode(pinInterrupteur,     INPUT);
  digitalWrite(pinInterrupteur, HIGH);   // Activation pullup interne

  // Initialisation des broches des boutons
  pinMode(pinBoutonRouge,     INPUT);
  pinMode(pinBoutonVert,      INPUT);
  digitalWrite(pinBoutonRouge, HIGH);    // Activation pullup interne
  digitalWrite(pinBoutonVert,  HIGH);    // Activation pullup interne

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
  afficher(position);

  if (output > 200) {
    deplacementGauche(output);
  }
  else if (output < -200) {
    deplacementDroite(abs(output));
  }
  else {
    arretMoteur();
  }

}

/*void loop() {
  // loop de controle manuel (boutons)
  if (millis() > previousMillis + 300 )  {
    Serial.print("Position = ");
    Serial.println(position);
    previousMillis = millis();
  }

  stateBoutonRouge = !digitalRead(pinBoutonRouge);   // On inverse la lecture car on veut le moteur arrete quand la pin est à l'état haut
  stateBoutonVert  = !digitalRead(pinBoutonVert);    // On inverse la lecture car on veut le moteur arrete quand la pin est à l'état haut

  analogWrite(pinPWM, vitesse);                           // Envoi de la vitesse sur la pin PWM

  // Test des boutons pour deplacement manuel (fin de courses logicielles)
  if ((stateBoutonVert == HIGH) and (position > 0)) {
    deplacementDroite();
  }
  else if ((stateBoutonRouge == HIGH) and (position < 6000)) {
    deplacementGauche();
  }
  else {
    arretMoteur();
  }


  }*/

// ---------------------------------------------------------------------------------------------------------------------------
void homing(int vitesseHoming) {
  // On Fait avancer le chariot vers la droite tant que l'interrupteur a le status 0. On arrete le moteur quand l'interrupteur passe au status 1
  analogWrite(pinPWM, homingSpeed);            // Envoi de la vitesse de homing sur la pin PWM
  while (digitalRead(pinInterrupteur) == HIGH) {
    deplacementDroite(vitesseHoming);
  }
  arretMoteur();
  delay(300);

  while (digitalRead(pinInterrupteur) == LOW) {
    deplacementGauche(vitesseHoming);
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
  // Deplacement vers la droite du chariot
  analogWrite(pinPWM,   vitesse);            // Envoi de la vitesse sur la pin PWM pinPWM
  digitalWrite(pinMoteur1, HIGH);
  digitalWrite(pinMoteur2,  LOW);
}
// ---------------------------------------------------------------------------------------------------------------------------
/*void deplacementProportionnel(k) {
  // Deplacement du chariot proportionnel a k
  if (k > 0) {
    analogWrite(pinPWM, vitesse);
    deplacementGauche();
  }
  else if (k < 0) {
    analogWrite(pinPWM, vitesse);
    deplacementDroite();
  }
  else {
    analogWrite(pinPWM, 0);
    arretMoteur();
  }

  }*/
// ---------------------------------------------------------------------------------------------------------------------------
void arretMoteur() {
  // Arret du moteur
  analogWrite(pinPWM,        0);
  digitalWrite(pinMoteur1, LOW);
  digitalWrite(pinMoteur2, LOW);
}
// ---------------------------------------------------------------------------------------------------------------------------
void afficher(volatile double variable) {
  // Affichage sur le port série toutes les period de la variable
  maintenant = millis();
  if ( (maintenant - dernierAffichage) >= periodeAffichage )  {
    Serial.println(variable);
    dernierAffichage = maintenant;
  }
}
// ---------------------------------------------------------------------------------------------------------------------------
void interruptionCodeurA() {
  stateCodeurA = (PIND & B00000100) >> 2;  // Lecture de la broche 2 en binaire ( equivaut a stateCodeurA = digitalRead(pinCodeurA); )
  stateCodeurB = (PIND & B00001000) >> 3;  // Lecture de la broche 3 en binaire ( equivaut a stateCodeurB = digitalRead(pinCodeurB); )

  if (stateCodeurA == HIGH) {              // Found a low-to-high on A phase. if(digitalRead(pinCodeurA)==HIGH){ .... read PE4
    if (stateCodeurB == HIGH) {             // Check B phase to see which way. if(digitalRead(pinCodeurB)==LOW) { .... read PH5
      position -- ;           // CCW
      //sens = false;
    }
    else {
      position ++ ;                      // CW
      //sens = true;
    }
  }
  else {
    if (stateCodeurB == HIGH) {             // Check B phase to see which way. if(digitalRead(pinCodeurB)==LOW) { .... read PH5
      position ++ ;                       // CCW
      //sens = true;
    }
    else {
      position -- ;                       // CW
      //sens = false;
    }
  }
}
// ---------------------------------------------------------------------------------------------------------------------------
void interruptionCodeurB() {
  stateCodeurA = (PIND & B00000100) >> 2;  // Lecture de la broche 2 en binaire ( equivaut a stateCodeurA = digitalRead(pinCodeurA); )
  stateCodeurB = (PIND & B00001000) >> 3;  // Lecture de la broche 3 en binaire ( equivaut a stateCodeurB = digitalRead(pinCodeurB); )

  if (stateCodeurA == HIGH) {              // Found a low-to-high on A phase. if(digitalRead(pinCodeurA)==HIGH){ .... read PE4
    if (stateCodeurB == HIGH) {             // Check B phase to see which way. if(digitalRead(pinCodeurB)==LOW) { .... read PH5
      position ++ ;           // CCW
      //sens = false;
    }
    else {
      position -- ;                      // CW
      //sens = true;
    }
  }
  else {
    if (stateCodeurB == HIGH) {             // Check B phase to see which way. if(digitalRead(pinCodeurB)==LOW) { .... read PH5
      position -- ;                       // CCW
      //sens = true;
    }
    else {
      position ++ ;                       // CW
      //sens = false;
    }
  }
}
// ---------------------------------------------------------------------------------------------------------------------------
