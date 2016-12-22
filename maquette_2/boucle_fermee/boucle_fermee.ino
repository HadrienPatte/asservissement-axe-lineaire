/* -------------------------------------------------------------------------------------------------------------------------
  Pilotage moteur DC avec 2 boutons et lecture position

  Le bouton de gauche (rouge) fait tourner le moteur dans un sens, l'autre bouton (vert) dans l'autre sens

  Un interrupteur gere une fin de course et l'initialisation au homing (interrupteur NC, ferme, donc 0, car connecte à GND par defaut)

  Les codeurs sont lus en binaire (lecture rapide) et sur les fronts montants ET descendants

  Implementation d'un asservissement proportionel

  V 2  Hadrien Patte 22/12/2016
  ----------------------------------------------------------------------------------------------------------------------------*/
/* -------------------------------------------------------------------------------------------------------------------------
  Notes : pas de pulldown internes sur des arduino, uniquement des pullup
          volatile pourles variables modifiables en interruption
          const pour les pin
  ----------------------------------------------------------------------------------------------------------------------------*/

volatile long position = 0;             // Position sent back by the encoder

long previousMillis = 0;                       // will store last time LED was updated


// Connexion du driver moteur L298N aux broches numeriques Arduino
const int pinPWM             =  9;      // Pin PWM enable
const int pinMoteur1         =  10;     // Pin Moteur 1
const int pinMoteur2         =  11;     // Pin Moteur 2

const int pinCodeurA         =   2;     // Broche signal codeur A
const int pinCodeurB         =   3;     // Broche signal codeur B
int stateCodeurA             =   0;     // Variable d'etat du codeur A
int stateCodeurB             =   0;     // Variable d'etat du codeur B

const int pinBoutonRouge     =   5;     // Broche signal bouton rouge
const int pinBoutonVert      =   6;     // Broche signal bouton vert
int stateBoutonRouge         =   1;     // Variable d'etat du bouton rouge
int stateBoutonVert          =   1;     // Variable d'etat du bouton vert

const int pinInterrupteur    =  12;     // Broche de l'interrupteur de fin de course et d'initialisation de la position

int vitesse                  = 255;     // Vitesse PWM du moteur (map 0-255)
int homingSpeed              = 255;     // Vitesse de homing

const int pinTest            =  13;

void setup() {
  Serial.begin(115200);                 // Activation du moniteur serie

  // Initialisation des broches de commande du moteur
  pinMode(pinPWM,     OUTPUT);
  pinMode(pinMoteur1, OUTPUT);
  pinMode(pinMoteur2, OUTPUT);

  // Initialisation des broches des codeurs
  pinMode(pinCodeurA, INPUT);
  pinMode(pinCodeurB, INPUT);

  // Initialisation de la broche de test
  pinMode(pinTest, OUTPUT);

  // Initialisation de la broche de l'interrupteur
  pinMode(pinInterrupteur,      INPUT);
  digitalWrite(pinInterrupteur, HIGH);  // Activation pullup interne

  // Initialisation des broches des boutons
  pinMode(pinBoutonRouge,      INPUT);
  pinMode(pinBoutonVert,       INPUT);
  digitalWrite(pinBoutonRouge, HIGH);  // Activation pullup interne
  digitalWrite(pinBoutonVert,  HIGH);  // Activation pullup interne

  // Initialise la position
  homing();
  attachInterrupt(0, doEncoderMotor, CHANGE); // sur le codeur A en 0 et B en 1

}

void loop() {

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
  else if ((stateBoutonRouge == HIGH) and (position < 3200)) {
    deplacementGauche();
  }
  else {
    arretMoteur();
  }

  /*
    if (((PIND & B00001000) >> 3) == HIGH) {
      digitalWrite(pinTest, HIGH);
    }
    else {
      digitalWrite(pinTest, LOW);
    }
  */
}

// ---------------------------------------------------------------------------------------------------------------------------
void homing() {
  // On Fait avancer le chariot vers la droite tant que l'interrupteur a le status 0. On arrete le moteur quand l'interrupteur passe au status 1
  analogWrite(pinPWM, homingSpeed);            // Envoi de la vitesse de homing sur la pin PWM
  while (digitalRead(pinInterrupteur) == HIGH) {
    deplacementDroite();
  }
  arretMoteur();
  delay(300);

  while (digitalRead(pinInterrupteur) == LOW) {
    deplacementGauche();
  }
  arretMoteur();
  delay(300);
}

// ---------------------------------------------------------------------------------------------------------------------------
void deplacementGauche() {
  // Déplacement vers la gauche du chariot
  digitalWrite(pinMoteur1, LOW);
  digitalWrite(pinMoteur2, HIGH);
}

// ---------------------------------------------------------------------------------------------------------------------------
void deplacementDroite() {
  // Déplacement vers la droite du chariot
  digitalWrite(pinMoteur1, HIGH);
  digitalWrite(pinMoteur2, LOW);
}

// ---------------------------------------------------------------------------------------------------------------------------
void arretMoteur() {
  // Arret du moteur
  digitalWrite(pinMoteur1, LOW);
  digitalWrite(pinMoteur2, LOW);
}

// ---------------------------------------------------------------------------------------------------------------------------
void doEncoderMotor() {
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
