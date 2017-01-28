/* -------------------------------------------------------------------------------------------------------------------------
  Pilotage moteur DC avec 2 boutons et un potentiometre

  Deux ILS gèrent les fin de course (interrupteur NC, ferme, donc 0, car connecte à GND par defaut)
  Le homing se fait sur l'ILS gauche

  Implementation d'un asservissement proportionel
  Implementation de la vitesse instantanee par integration de la position

  V 2  Hadrien Patte 28/01/2017
  ----------------------------------------------------------------------------------------------------------------------------*/
/* -------------------------------------------------------------------------------------------------------------------------
  Notes : * pas de pulldown internes sur des arduino, uniquement des pullup
            volatile pour les variables modifiables en interruption
            const pour les pin
  ----------------------------------------------------------------------------------------------------------------------------*/

volatile long position = 0;             // position mesuree par le codeur

long previousMillis = 0;                // pour l'affichage de la position sur le port serie


// Connexion du driver moteur L298N aux broches numeriques Arduino
const int pinPWM          =  12;     // Pin PWM enable
const int pinMoteur1          =  11;     // Pin Moteur 1
const int pinMoteur2          =  10;     // Pin Moteur 2

const int pinCodeurA         =   2;     // Broche signal codeur A
const int pinCodeurB         =   3;     // Broche signal codeur B
int stateCodeurA             =   0;     // Variable d'etat du codeur A
int stateCodeurB             =   0;     // Variable d'etat du codeur B


const int ilsDroitPin  =  14;     // Broche signal de l'ILS droit
const int ilsGauchePin =  15;     // Broche signal de l'ILS gauche (celui qui sert pour le homing)
int homingSpeed        = 125;     // Vitesse de homing

int vitesse            = 150;

long consigne          = 5000;

volatile bool sens = false;

float vitesseInstantanee = 0;
long previousposition = 0;

int period = 50;

void setup() {
  Serial.begin(115200);           // Activation de la communication sur le port série

  pinMode(pinPWM,      OUTPUT);
  pinMode(pinMoteur1,      OUTPUT);
  pinMode(pinMoteur2,      OUTPUT);

  pinMode(pinCodeurA,  INPUT);
  pinMode(pinCodeurB,  INPUT);
  //digitalWrite(pinCodeurA, HIGH);  // Activation pullup interne
  //digitalWrite(pinCodeurB, HIGH);  // Activation pullup interne


  pinMode(ilsDroitPin,  INPUT);          // configure la broche ILS en entrée
  pinMode(ilsGauchePin, INPUT);          // configure la broche ILS en entrée
  digitalWrite(ilsDroitPin,  HIGH);      // Activation pullup interne
  digitalWrite(ilsGauchePin, HIGH);      // Activation pullup interne

  homing();

  attachInterrupt(0, doEncoderMotor, CHANGE);   // Execute doEncoderMotor function on interrupt 0 - pin 2
  // triggered by a "CHANGE". Means RISING ou FALLING edge

  previousposition = position;
  //deplacementPosition(consigne);
}

void loop() {


  // Print encoder position every "period" through the serial port
  if (millis() > previousMillis + period )  {
    //Serial.println(sens);
    if (vitesseInstantanee != 0.0) {
      //Serial.print("vitesseInstantanee = ");
      Serial.println(vitesseInstantanee);
    }
    vitesseInstantanee = 1000 * (position - previousposition) / period;
    previousposition = position;
    previousMillis = millis();
  }

  /*// Test de fin de course
    if ((digitalRead(ilsGauchePin) == 1) or (digitalRead(ilsDroitPin) == 1)) {
    digitalWrite(pinMoteur1, LOW);            // Arret du moteur
    digitalWrite(pinMoteur2, LOW);
    }*/
  // integration de la position pour avoir la vitesse instantanee

  analogWrite(pinPWM, vitesse);                           // Envoi de la vitesse sur la pin PWM pinPWM
  /*while (abs(consigne - position) > 10) {
    if (consigne > position) {
      deplacementDroite();
    }
    else {
      deplacementGauche();
    }
    }*/
  if (consigne > position + 10) {
    deplacementDroite();
  }
  else if (consigne < position - 10) {
    deplacementGauche();
  }
  else {
    arretMoteur();
  }
  /*
    if (consigne - 10 > position) {
      deplacementDroite();
    }
    else if (consigne + 10 < position) {
      deplacementGauche();
    }
    else {
      arretMoteur();
    }*/

}

// ---------------------------------------------------------------------------------------------------------------------------
void homing() {
  // On Fait avancer le chariot vers la gauche tant que l'ILS gauche a le status 0. On arrete le moteur quand l'ILS passe au status 1
  analogWrite(pinPWM, homingSpeed);            // Envoi de la vitesse sur la pin PWM pinPWM
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
void deplacementPosition(long consigne) {
  // deplacement jusqu a la position consigne avec retroaction
  while (abs(consigne - position) > 10) {
    if (consigne > position) {
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
