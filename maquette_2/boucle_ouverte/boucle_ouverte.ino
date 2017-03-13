/* -------------------------------------------------------------------------------------------------------------------------
  Pilotage moteur DC avec 2 boutons et lecture position

  Le bouton de gauche (rouge) fait tourner le moteur dans un sens, l'autre bouton (vert) dans l'autre sens

  Un interrupteur gere une fin de course et l'initialisation au homing (interrupteur NC, ferme, donc 0, car connecte a GND par defaut)

  Les codeurs sont lus en binaire (lecture rapide) et sur les fronts montants ET descendants des deux codeurs ( 2 interruptions )

  Asservissement PID implemente intrinsequement precis

  La course est de 6647 impulsions

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
float vitesseinstantannee = 0;
volatile float position = 0; // input
volatile float previousposition = 0; // input
float dernierePosition = 0;
float consigne = 3000; // setpoint
float sortiePID = 0; // output
float ITerm = 0;
int periodePID = 10;
int vitesseMin = 200;
int vitesseMax = 255;

// Coefficients PID
float kp = 1;
float ki = 0;
float kd = 0;

// Connexion du driver moteur L298N aux broches numeriques Arduino
const byte pinPWM             =   9;     // Pin PWM enable
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

int homingSpeed               = 230;     // Vitesse de homing (0-255)

void setup() {
  Serial.begin(115200);                  // Activation du moniteur serie

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
  ki = ki * ((float)periodePID) / 1000;
  kd = kd * 1000 / ((float)periodePID);
}

void loop() {
  // boucle de controle asservi (reponse a une consigne)
  afficher();

  //calculPID();

  deplacementGauche(255);

}

/*void loop() {
  // loop de controle manuel (boutons)
  afficher(position);

  stateBoutonRouge = !digitalRead(pinBoutonRouge);   // On inverse la lecture car on veut le moteur arrete quand la pin est a l'etat haut
  stateBoutonVert  = !digitalRead(pinBoutonVert);    // On inverse la lecture car on veut le moteur arrete quand la pin est a l'etat haut

  // Test des boutons pour deplacement manuel (fin de courses logicielles)
  if ((stateBoutonVert == HIGH) and (position > 0)) {
    deplacementDroite(vitesseMax);
  }
  else if ((stateBoutonRouge == HIGH) and (position < 6000)) {
    deplacementGauche(vitesseMax);
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
void afficher() {
  // Affichage sur le port serie toutes les period de la variable
  maintenant = millis();
  if ( (maintenant - dernierAffichage) >= periodeAffichage )  {
    vitesseinstantannee = (position - previousposition) / periodeAffichage;
    previousposition = position;
    Serial.println(vitesseinstantannee);
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
  stateCodeurA = (PIND & B00000100) >> 2;  // Lecture de la broche 2 en binaire ( equivaut a stateCodeurA = digitalRead(pinCodeurA); )
  stateCodeurB = (PIND & B00001000) >> 3;  // Lecture de la broche 3 en binaire ( equivaut a stateCodeurB = digitalRead(pinCodeurB); )

  if (stateCodeurA == HIGH) {
    if (stateCodeurB == HIGH) {
      position -- ;
    }
    else {
      position ++ ;
    }
  }
  else {
    if (stateCodeurB == HIGH) {
      position ++ ;
    }
    else {
      position -- ;
    }
  }
}

// ---------------------------------------------------------------------------------------------------------------------------
void interruptionCodeurB() {
  stateCodeurA = (PIND & B00000100) >> 2;  // Lecture de la broche 2 en binaire ( equivaut a stateCodeurA = digitalRead(pinCodeurA); )
  stateCodeurB = (PIND & B00001000) >> 3;  // Lecture de la broche 3 en binaire ( equivaut a stateCodeurB = digitalRead(pinCodeurB); )

  if (stateCodeurA == HIGH) {
    if (stateCodeurB == HIGH) {
      position ++ ;
    }
    else {
      position -- ;
    }
  }
  else {
    if (stateCodeurB == HIGH) {
      position -- ;
    }
    else {
      position ++ ;
    }
  }
}

// ---------------------------------------------------------------------------------------------------------------------------
