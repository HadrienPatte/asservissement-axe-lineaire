/* -------------------------------------------------------------------------------------------------------------------------
  Pilotage moteur DC avec 2 boutons et lecture position

  Le bouton de gauche (rouge) fait tourner le moteur dans un sens, l'autre bouton (vert) dans l'autre sens

  Un interrupteur gere une fin de course et l'initialisation au homing (interrupteur NC, ferme, donc 0, car connecte à GND par defaut)

  Test en cours des interruptions pour récpérer les informations du codeur

  V 1  Hadrien Patte 22/12/2016
  ----------------------------------------------------------------------------------------------------------------------------*/

volatile long encoderPosition = 0;             // Position sent back by the encoder

long previousMillis = 0;                       // will store last time LED was updated


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

const int interrupteur =  12;     // Broche de l'interrupteur de fin de course
int testPin = 13;
int vitesse            =   0;     // Vitesse lue sur le potentiometre
int homingSpeed        = 255;     // Vitesse de homing

int stateA = 0;
int stateB = 0;
void setup() {
  Serial.begin(115200);

  pinMode(enA,      OUTPUT);
  pinMode(in1,      OUTPUT);
  pinMode(in2,      OUTPUT);

  pinMode(encoderPinA,  INPUT);
  pinMode(encoderPinB,  INPUT);

  pinMode(interrupteur, INPUT);          // configure la broche ILS en entrée
  digitalWrite(interrupteur,  HIGH);      // Activation pulldown interne

  pinMode(rougePin, INPUT);
  pinMode(vertPin,  INPUT);
  digitalWrite(rougePin, HIGH);  // Activation pullup interne
  digitalWrite(vertPin,  HIGH);  // Activation pullup interne
  pinMode(testPin, OUTPUT);
  homing();
  attachInterrupt(0, doEncoderMotor, CHANGE); // sur le codeur A en 0 et B en 1

}

void loop() {

  if (millis() > previousMillis + 300 )  {
    //Serial.println(sens);
    Serial.print("Encoder position = ");
    Serial.println(encoderPosition);
    previousMillis = millis();
  }

  if (digitalRead(interrupteur) == LOW) {
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
  if ((vertState == HIGH) and (encoderPosition > 0)) {
    deplacementDroite();
  }
  else if ((rougeState == HIGH) and (encoderPosition < 3200)) {
    deplacementGauche();
  }
  else {
    arretMoteur();
  }

}


// ---------------------------------------------------------------------------------------------------------------------------
void homing() {
  // On Fait avancer le chariot vers la gauche tant que l'ILS gauche a le status 0. On arrete le moteur quand l'ILS passe au status 1
  analogWrite(enA, homingSpeed);            // Envoi de la vitesse sur la pin PWM enA
  while (digitalRead(interrupteur) == HIGH) {
    deplacementDroite();
  }
  arretMoteur();
  delay(300);

  while (digitalRead(interrupteur) == LOW) {
    deplacementGauche();
  }
  arretMoteur();
  delay(300);
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
void doEncoderMotor() {
  //stateA = (PINE & B00010000) >> 4;
  //stateB = (PINH & B00100000) >> 5;   //que pour la pin 8
  stateA = digitalRead(encoderPinA);
  stateB = digitalRead(encoderPinB);

  if (stateA == HIGH) {              // Found a low-to-high on A phase. if(digitalRead(encoderPinA)==HIGH){ .... read PE4
    if (stateB == HIGH) {             // Check B phase to see which way. if(digitalRead(encoderPinB)==LOW) { .... read PH5
      encoderPosition -- ;           // CCW
      //sens = false;
    }
    else {
      encoderPosition ++ ;                      // CW
      //sens = true;
    }
  }
  else {
    if (stateB == HIGH) {             // Check B phase to see which way. if(digitalRead(encoderPinB)==LOW) { .... read PH5
      encoderPosition ++ ;                       // CCW
      //sens = true;
    }
    else {
      encoderPosition -- ;                       // CW
      //sens = false;
    }
  }
}
