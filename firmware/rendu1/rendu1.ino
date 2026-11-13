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


///////////////////////////////////////////declaration des variables utiles
const uint16_t maxSpeed = 400;
// This is the maximum speed the motors will be allowed to turn.
// A maxSpeed of 400 lets the motors go at top speed.  Decrease
// this value to impose a speed limit.

#define NUM_SENSORS 5
unsigned int lineSensorValues[NUM_SENSORS];

uint16_t batterie;
char incomingByte;
int positionl;

unsigned int seuil=700;
unsigned int capteur0=lineSensorValues[0];
unsigned int capteur1=lineSensorValues[1];
unsigned int capteur2=lineSensorValues[2];
unsigned int capteur3=lineSensorValues[3];
unsigned int capteur4=lineSensorValues[4];

unsigned char etat; //variable d'état de l'automate
unsigned char etat_actuel, etat_precedent;
//unsigned char first_actuel=0, first_precedent=0;
unsigned char isFirst=1;
unsigned char compt_avant_stop=0;

int16_t offsetL=0,offsetR=0;
int16_t tick_moteurL=0,tick_moteurR=0;
int16_t speedR,speedL;

unsigned long originTime=0,currentTime=0,temps_affiche;


//transforme le tableau en un tableau de bits
void traduction(unsigned int *p)
{
      unsigned char i;
      for(i=0;i<NUM_SENSORS;i++)
      {
            if(*(p+i)>(lineSensors.calibratedMinimumOn[i]*2+
                    lineSensors.calibratedMaximumOn[i])/3)
            {
                  *(p+i)=1;
            }
            else
            {
                  *(p+i)=0;
            }
      }
}





//*****************************************************
// SETUP initialisations
//****************************************************


void setup()
{
      // put your setup code here, to run once:
      buzzer.play("L30 cdeg");
      buttonB.waitForButton(); // on attend d'appuyer sur le bouton B



      Serial1.begin(38400); //initialisation bluetooth
      Wire.begin(); //I2C


      delay(800); //attendre pour ne pas risquer de blesser le doigt*/
      Serial1.println("Bonjour groupe 15"); //envoie le string au bluetooth

      if (!motion.calibrateGyro() || !motion.calibrateLineSensors(lineSensors)) {
            ledRed(1);
            Serial1.println("Erreur de calibration");
            while (true) delay(100);
      }

      encoders.init();
      encoders.getCountsAndResetRight();
      encoders.getCountsAndResetLeft();
      if(encoders.checkErrorLeft()||encoders.checkErrorRight())
      {
            Serial1.println("erreur encodeurs");
      }
      else
      {
            Serial1.println("pas d'erreur");
      }
 }


///////////////////////////////////////////////////////



///////////////BOUCLE/////////////////////
void loop() {
          // put your main code here, to run repeatedly

          //delay(1000);
          /*
           if(readBatteryMillivolts()<5100)
           {
            Serial1.println("pas assez de batterie");
            while(1)
            {
              motors.setSpeeds(0,0);
            }
           }
           */
          positionl = lineSensors.readLine(lineSensorValues)-2000;

          traduction(lineSensorValues);

           /*
          capteur0=lineSensorValues[0];
          capteur1=lineSensorValues[1];
          capteur2=lineSensorValues[2];
          capteur3=lineSensorValues[3];
          capteur4=lineSensorValues[4];


          Serial1.println("**valeurs des capteurs**");
          Serial1.print(capteur0);
          Serial1.print(" ");
          Serial1.print(capteur1);
          Serial1.print(" ");
          Serial1.print(capteur2);
          Serial1.print(" ");
          Serial1.print(capteur3);
          Serial1.print(" ");
          Serial1.println(capteur4);
          */



          //actualisation de l'état des capteurs ( est-ce que tout les capteurs sont sur la ligne ou non)
          etat_actuel=lineSensorValues[0]&&lineSensorValues[1]&&lineSensorValues[2]&&lineSensorValues[3]&&lineSensorValues[4];

          //passage d'une ligne noir à une ligne blanche
          if(etat_precedent&&(!etat_actuel)){
                    currentTime=millis();

                    if(isFirst)
                    {
                        originTime=currentTime;
                        isFirst=0;
                        tick_moteurL=0;
                        tick_moteurR=0;
                        compt_avant_stop=0;
                        temps_affiche=currentTime-originTime;
                        Serial1.println("premier passage");
                        Serial1.print("offset Moteur Gauche : ");
                        Serial1.print(offsetL);
                        Serial1.print(" --- offset Moteur Droit : ");
                        Serial1.println(offsetR);
                        Serial1.print(" temps : ");
                        Serial1.println(temps_affiche);
                    }
                    else if(compt_avant_stop<5)
                    {
                      tick_moteurL+=encoders.getCountsAndResetLeft();
                      tick_moteurR+=encoders.getCountsAndResetRight();
                      temps_affiche=currentTime-originTime;
                      compt_avant_stop+=1;
                      Serial1.print("Tick Moteur Gauche : ");
                      Serial1.print(tick_moteurL);
                      Serial1.print(" --- Tick Moteur Droit : ");
                      Serial1.print(tick_moteurR);
                      Serial1.print(" temps : ");
                      Serial1.println(temps_affiche);

                    }

          }
          if(compt_avant_stop>=4)
                    {
                          temps_affiche=currentTime - originTime;
                          //delay(500);
                          motors.setSpeeds(0,0);
                          //Serial1.print("Temps total : ");
                          //Serial1.println(temps_affiche);
                    }
          etat_precedent=etat_actuel;


          if(Serial1.available()){
                    incomingByte = Serial1.read();
                    switch(incomingByte){

                              case 'e':
                                        batterie=readBatteryMillivolts();
                                        Serial1.print("Niveau de batterie : ");
                                        Serial1.print(batterie);
                                        Serial1.println(" mV");
                                        break;
                              case 'k':
                                        Serial1.println("test buzzer");
                                        buzzer.play("L16 cdegreg4");
                                        break;

                              case 'a':
                                        Serial1.println("**Avancer**");
                                        speedR=300;
                                        speedL=speedR+4; //offset moteur (experience)
                                        motors.setSpeeds(speedR,speedR);
                                        break;
                              case 's':
                                        Serial1.println("**Arreter**");
                                        motors.setSpeeds(0,0);
                                        break;
                              default :
                                       // motors.setSpeeds(0,0);
                                        break;
                  }

          if (incomingByte == 'L' || incomingByte == 'R') { // Si c'est une commande moteur
              int speedValue = Serial1.parseInt(); // Lire la valeur numérique qui suit


              if (incomingByte == 'L') {
                  motors.setSpeeds(speedValue, speedR); // Modifier seulement la vitesse gauche
              }
              else if (incomingByte == 'R') {
                  motors.setSpeeds(speedL, speedValue); // Modifier seulement la vitesse droite
              }
    }


          }
}
