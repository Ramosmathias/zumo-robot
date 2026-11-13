

#include <Wire.h>
#include <Zumo32U4.h>
#include <ZumoMotion.h>
#include <math.h>

Zumo32U4Buzzer buzzer;
Zumo32U4LineSensors lineSensors;
Zumo32U4Motors motors;
Zumo32U4ButtonB buttonB;
Zumo32U4Encoders encoders;
Zumo32U4IMU imu;
ZumoMotion motion(imu);
Zumo32U4ProximitySensors proxSensors;  // ajout du capteur de proximité avant gauche et droit

#define NUM_SENSORS 5
unsigned int lineSensorValues[NUM_SENSORS];

////////////////////// declaration des variables utiles //////////////////////

//distances entre robot et ligne
long distdroit, distgauche;
int positionl;

//temporel
unsigned long ll, lastll;

// variable etat de l'automate
unsigned char etat = 0;
uint8_t count_rapprochement = 0, count_prox_sensor = 0, count_noir = 0 ;
uint16_t batterie;
char incomingByte;
char go = 0;

//capteurs de ligne
unsigned char sens_stat;

//variable obstacle
uint8_t proxR, proxL;
uint8_t premiere_inter = 0;

//variable PID
unsigned char Kp, Kp_g;
unsigned char Kd, Kd_g;
unsigned char Ki, Ki_g;
int32_t error, error_g;
int32_t lastError = 0, lastError_g = 0;
int32_t integral = 0, integral_g = 0;
int16_t deriv, deriv_g;
int16_t speedDifference;

//variable de la vitesse de nos roues:
/*const*/ uint16_t maxSpeed = 400;
uint16_t Speed;
int16_t rightSpeed = 100;
int16_t leftSpeed = rightSpeed + 4;


////////////////////// FONCTIONS //////////////////////

//renvoie la valeur des capteurs (entre 0 et 1000)
unsigned char  Analyse(const unsigned int *p) {
                unsigned char i;
                unsigned char ret = 0;
                for (i = 0; i < 5; i++) {
                                ret = ret << 1;
                                ret = ret + (p[i] > (lineSensors.calibratedMinimumOn[i] * 2 + lineSensors.calibratedMaximumOn[i]) / 3 ? 1 : 0);
                }
                return ret;
}

// Asservissement sur ligne noire
void PID() {
                error = positionl;
                deriv = error - lastError;
                integral += error;
                Kp = 2;
                Ki = 1;
                Kd = 5;
                speedDifference = Kp * error / 5  + Ki * integral / 200 + Kd * deriv / 3;
                lastError = error;
                leftSpeed = (int16_t)Speed + speedDifference;
                rightSpeed = (int16_t)Speed - speedDifference;
                leftSpeed = constrain(leftSpeed, 0, (int16_t)maxSpeed);
                rightSpeed = constrain(rightSpeed, 0, (int16_t)maxSpeed);
                motors.setSpeeds(leftSpeed, rightSpeed);
}

//Asservissement sur centrale inertielle
void PID_GYRO() {
                const int32_t microdegreesPerErrorUnit = 67055L;
                error_g = motion.headingMicrodegrees() / microdegreesPerErrorUnit;
                deriv_g = error_g - lastError_g;
                integral_g += error_g;
                Kp_g = 2;
                Ki_g = 1;
                Kd_g = 5;
                speedDifference = Kp_g * (error_g / 5) + Ki_g * (integral_g / 200) + Kd_g * (deriv_g / 3);
                lastError_g = error_g;
                leftSpeed = (int16_t)Speed + speedDifference;
                rightSpeed = (int16_t)Speed - speedDifference;
                leftSpeed = constrain(leftSpeed, 0, (int16_t)maxSpeed);
                rightSpeed = constrain(rightSpeed, 0, (int16_t)maxSpeed);
                motors.setSpeeds(leftSpeed, rightSpeed);
}

void automate() {
                count_prox_sensor += 1;
                positionl = lineSensors.readLine(lineSensorValues) - 2000;
                sens_stat = Analyse(lineSensorValues);
                if (sens_stat == 4) {
                                motion.resetHeading();
                }
                distdroit = distdroit + encoders.getCountsAndResetRight();
                distgauche = distgauche + encoders.getCountsAndResetLeft();
                if (count_prox_sensor == 7) {
                                proxSensors.read();
                                proxR = proxSensors.countsFrontWithRightLeds(); //capteur proximité droite
                                proxL = proxSensors.countsFrontWithLeftLeds(); //capteur proximité gauche
                                count_prox_sensor = 0;
                }
                switch (etat) {
                                // Etat 0 : initial
                                case 0 :
                                                Serial1.println("Etat 0 : initial");
                                                Serial1.println("(E0) Pret a partir...");
                                                delay(200);
                                                distdroit = 0;
                                                distgauche = 0;
                                                etat = 1;
                                                break;
                                //etat 1 :  revenir sur la ligne après calibrage
                                case 1:
                                                positionl = lineSensors.readLine(lineSensorValues) - 2000;
                                                sens_stat = Analyse(lineSensorValues); //acquisition

                                                switch (sens_stat)
                                                {
                                                                case 2:
                                                                                motors.setSpeeds(104, -100);
                                                                                etat = 1;
                                                                                break;
                                                                case 4:
                                                                                motors.setSpeeds(0, 0);
                                                                                etat = 2;
                                                                                break;
                                                                case 6:
                                                                                motors.setSpeeds(104, -100);
                                                                                etat = 1;
                                                                                break;
                                                                case 8:
                                                                                motors.setSpeeds(-104, 100);
                                                                                etat = 1;
                                                                                break;
                                                                case 12 :
                                                                                motors.setSpeeds(-104, 100);
                                                                                etat = 1;
                                                                                break;
                                                                default:
                                                                                motors.setSpeeds(-204, 200);
                                                                                delay(100);
                                                                                etat = 1;
                                                                                break;
                                                }
                                                break;
                                //Etat 2 : Attente disparition obstacle
                                case 2:
                                                proxSensors.read();
                                                proxR = proxSensors.countsFrontWithRightLeds(); //capteur proximité droite
                                                proxL = proxSensors.countsFrontWithLeftLeds(); //capteur proximité gauche
                                                if ((proxR >= 6) || (proxL >= 6)) {
                                                                motors.setSpeeds(0, 0); //arreter le robot car obstacle
                                                                Serial1.println("(E2) Obstacle");
                                                                etat = 2;
                                                }
                                                if ((proxR < 5) || (proxL < 5))
                                                {
                                                                etat = 3;
                                                }
                                                break;
                                // Etat 3 : Suivi de ligne
                                case 3 :
                                                Speed = 400;
                                                PID();
                                                if (sens_stat > 14) //tout noir
                                                {
                                                                count_noir += 1;
                                                                if (count_noir >= 15) // arrive sur le parking  //depend de la periode d'echantillonnage
                                                                {
                                                                                etat = 5;
                                                                }
                                                }
                                                if ( !(sens_stat & 0x10) && !(sens_stat & 0x01) )
                                                {
                                                                if ( (count_noir != 0) && (count_noir < 15)) //intersection possible enlever 2 partie
                                                                {
                                                                                //buzzer.play("!T130 O4 V10 L4 MS g.g.g8e8>c."); //Pokemon
                                                                                buzzer.play("!T220 O4 L4 <a8c8ddd8e8fff8g8eed8c8c8d8"); //Pirate
                                                                                //buzzer.play("L28 cdeg");
                                                                                premiere_inter = 1;
                                                                }
                                                                count_noir = 0;
                                                }
                                                if ((proxR >= 6) || (proxL >= 6))
                                                {
                                                                if (premiere_inter == 1)
                                                                {
                                                                                etat = 7;
                                                                }
                                                }
                                                if (sens_stat == 0) //tout blanc
                                                {
                                                                etat = 4;
                                                }
                                                break;
                                // Etat 4 : Centrale inertielle
                                case 4 :
                                                Speed = 400;
                                                //Speed = 300;
                                                if (!motion.updateHeading()) {
                                                                motors.setSpeeds(0, 0);
                                                                etat = 6;
                                                                break;
                                                }
                                                PID_GYRO();
                                                if (sens_stat == 0)
                                                {
                                                                etat = 4;
                                                }
                                                if (sens_stat & 0x0E) //a mediter
                                                {
                                                                etat = 3;
                                                }
                                                break;
                                // Etat 5 : arret
                                case 5 :
                                                motors.setSpeeds(0, 0);
                                                Serial1.println("(E4) Fin du parcours");
                                                etat = 6;
                                                break;
                                // Etat 6 : Immobile
                                case 6 :
                                                etat = 6;
                                                break;
                                // Etat 7 : stop dans les 20 centimetres
                                case 7 :
                                                count_rapprochement += 1;
                                                if (count_rapprochement < 10)
                                                {
                                                                etat = 3;
                                                }
                                                if (count_rapprochement >= 10)
                                                {               count_rapprochement = 0;
                                                                etat = 8;
                                                }
                                                break;
                                case 8 :

                                                etat = motion.rotate(90, 400) ? 9 : 6;
                                                break;
                                case 9:
                                                //(153 * 13) / 5

                                                motors.setSpeeds(397, 153);
                                                //motors.setSpeeds(153 , (397);

                                                if (sens_stat > 14) //tout noir
                                                {
                                                                count_noir += 1;
                                                }
                                                if ( !(sens_stat & 0x10) && !(sens_stat & 0x01) )
                                                {
                                                                if ( (count_noir != 0) && (count_noir < 15)) //intersection possible enlever 2 partie
                                                                {
                                                                                etat = 10;
                                                                }
                                                                count_noir = 0;
                                                }
                                                else
                                                {
                                                                etat = 9;
                                                }
                                                break;
                                case 10 :
                                                etat = motion.rotate(90, 400) ? 1 : 6;
                                                break;
                                default :
                                                etat = 6;
                                                break;
                }
}
//*****************************************************
// SETUP initialisations
//****************************************************
void setup()
{
                // Play music and wait for it to finish before we start driving.
                buzzer.play("L28 V15 cdeg");
                //buzzer.play("!T220 O4 L4 <a8c8ddd8e8fff8g8eed8c8c8d8"); //Pirate
                //buzzer.play("!T130 O4 V10 L4 MS g8g8ge>c8."); //Pokemon
                while (buzzer.isPlaying());

                Serial1.begin(38400);
                Wire.begin();
                buttonB.waitForButton();
                delay(100);
                Serial1.println("Bonjour : ZU15 (RAMOS TASSAIN)");
                if (!motion.calibrateGyro() || !motion.calibrateLineSensors(lineSensors)) {
                                ledRed(1);
                                Serial1.println("Erreur de calibration");
                                while (true) delay(100);
                }

                proxSensors.initFrontSensor();// initialise les capteurs de proximité avant.


                // encoders.init(); meme chose, mais moins efficace
                encoders.getCountsAndResetRight();
                encoders.getCountsAndResetLeft();

                //motors.setSpeeds(LSpeed, RSpeed);

                positionl = lineSensors.readLine(lineSensorValues) - 2000;
                sens_stat = Analyse(lineSensorValues); //acquisition
}




void loop() {
                if (Serial1.available()) {
                                incomingByte = Serial1.read();
                                switch (incomingByte) {
                                                case 'e':
                                                                batterie = readBatteryMillivolts();
                                                                Serial1.print("Niveau de batterie : ");
                                                                Serial1.print(batterie);
                                                                Serial1.println(" mV");
                                                                //incomingByte = 0;
                                                                break;
                                                case 'g':
                                                                etat = 0;

                                                                maxSpeed = 400;
                                                                //incomingByte = 0;
                                                                break;
                                                case 's':
                                                                motors.setSpeeds(0, 0);
                                                                maxSpeed = 0;
                                                                //incomingByte = 0;
                                                                break;
                                                default :
                                                                motors.setSpeeds(0, 0);
                                                                break;
                                }
                }

                ll = micros();

                positionl = lineSensors.readLine(lineSensorValues) - 2000;
                sens_stat = Analyse(lineSensorValues); //acquisition

                if ((ll - lastll) >= 15000) //on test toutes les 20ms (30ms pour les proxSensors)
                {
                                lastll = ll;
                                automate();
                }
}
