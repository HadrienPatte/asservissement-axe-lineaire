/* -------------------------------------------------------------------------------------------------------------------------
  Pilotage moteur DC avec 2 boutons et un potentiometre

  Le bouton de gauche (rouge) fait tourner le moteur dans un sens, l'autre bouton (vert) dans l'autre sens
  Le potentiomètre permet de régler la vitesse
  Deux ILS gèrent les fin de course (interrupteur NC, ferme, donc 0, car connecte à GND par defaut)
  Le homing se fait sur l'ILS gauche


  Test en cours des interruptions pour récpérer les informations des codeurs du moteur

  ----------------------------------------------------------------------------------------------------------------------------*/

const int encoderPinA  = 2;                        // PE4
const int encoderPinB  = 3;                        // PH5

int stateA = 0;
int stateB = 0;

volatile bool sens = false;


volatile long encoderPosition = 0;             // Position sent back by the encoder

long previousMillis = 0;                       // will store last time LED was updated


// Connexion du driver moteur L298N aux broches numeriques Arduino
const int enA          =  12;     // Pin PWM enable
const int in1          =  11;     // Pin Moteur 1
const int in2          =  10;     // Pin Moteur 2

const int ilsDroitPin  =  14;     // Broche signal de l'ILS droit
const int ilsGauchePin =  15;     // Broche signal de l'ILS gauche (celui qui sert à l'homing)
int homingSpeed        = 125;     // Vitesse de homing

int vitesse            = 150;     // Vitesse lue sur le potentiometre

long consigne          = 7230;

void setup() {
  Serial.begin(115200);           // Activation de la communication sur le port série

  pinMode(enA,      OUTPUT);
  pinMode(in1,      OUTPUT);
  pinMode(in2,      OUTPUT);

  pinMode(encoderPinA,  INPUT);
  pinMode(encoderPinB,  INPUT);
  //digitalWrite(encoderPinA, HIGH);  // Activation pullup interne
  //digitalWrite(encoderPinB, HIGH);  // Activation pullup interne


  pinMode(ilsDroitPin,  INPUT);          // configure la broche ILS en entrée
  pinMode(ilsGauchePin, INPUT);          // configure la broche ILS en entrée
  digitalWrite(ilsDroitPin,  HIGH);      // Activation pullup interne
  digitalWrite(ilsGauchePin, HIGH);      // Activation pullup interne

  homing();

  attachInterrupt(0, doEncoderMotor, CHANGE);   // Execute doEncoderMotor function on interrupt 0 - pin 2
  // triggered by a "CHANGE". Means RISING ou FALLING edge

  deplacementPosition(consigne);
}

void loop() {


  // Print encoder position every "period" through the serial port
  if (millis() > previousMillis + 300 )  {
    //Serial.println(sens);
    Serial.print("Encoder position = ");
    Serial.println(encoderPosition);
    previousMillis = millis();
  }

  /*// Test de fin de course
    if ((digitalRead(ilsGauchePin) == 1) or (digitalRead(ilsDroitPin) == 1)) {
    digitalWrite(in1, LOW);            // Arret du moteur
    digitalWrite(in2, LOW);
    }*/


  analogWrite(enA, vitesse);                           // Envoi de la vitesse sur la pin PWM enA

/*
  if (consigne - 10 > encoderPosition) {
    deplacementDroite();
  }
  else if (consigne + 10 < encoderPosition) {
    deplacementGauche();
  }
  else {
    arretMoteur();
  }*/

}

// ---------------------------------------------------------------------------------------------------------------------------
void homing() {
  // On Fait avancer le chariot vers la gauche tant que l'ILS gauche a le status 0. On arrete le moteur quand l'ILS passe au status 1
  analogWrite(enA, homingSpeed);            // Envoi de la vitesse sur la pin PWM enA
  while (digitalRead(ilsGauchePin) == 0) {
    deplacementGauche();
  }
  arretMoteur();
  delay(300);

  while (digitalRead(ilsGauchePin) == 1) {
    deplacementDroite();
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

// ---------------------------------------------------------------------------------------------------------------------------
void deplacementPosition(long consigne) {
  // deplacement jusqu a la position consigne avec retroaction
  while (abs(consigne - encoderPosition) > 10) {
    if (consigne > encoderPosition) {
      deplacementDroite();
    }
    else {
      deplacementGauche();
    }
  }
  arretMoteur();
}

// ---------------------------------------------------------------------------------------------------------------------------

// Code for interrupt 0. Gives the encoder signed cumulated ticks, including CW and CCW (plus and minus) ways

void doEncoderMotor() {
  //stateA = (PINE & B00010000) >> 4;
  //stateB = (PINH & B00100000) >> 5;   //que pour la pin 8

  stateA = digitalRead(encoderPinA);
  stateB = digitalRead(encoderPinB);
  if (stateA == HIGH) {              // Found a low-to-high on A phase. if(digitalRead(encoderPinA)==HIGH){ .... read PE4
    if (stateB == LOW) {             // Check B phase to see which way. if(digitalRead(encoderPinB)==LOW) { .... read PH5
      encoderPosition -- ;           // CCW
      sens = false;
    }

    else {
      encoderPosition ++ ;                      // CW
      //sens = true;
    }
  }

  else {
    if (stateB == LOW) {             // Check B phase to see which way. if(digitalRead(encoderPinB)==LOW) { .... read PH5
      encoderPosition ++ ;                       // CCW
      sens = true;
    }

    else {
      encoderPosition -- ;                       // CW
      //sens = false;
    }
  }
}
// ---------------------------------------------------------------------------------------------------------------------------
