/***************************************************************/
/************************Bibliotheques**************************/
/***************************************************************/


#include <avr/sleep.h>
#include <avr/wdt.h>
#include <avr/power.h>
#include <RTClib.h>
#include "DHT.h"
#include <SoftwareSerial.h>

/***************************************************************/
/*****************Constantes & Parametres***********************/
/***************************************************************/

/* Parametres gain autonomie de batterie */
int samplingTime = 280;
int deltaTime = 40;
int sleepTime = 9680;

/* Parametre capteur PM */
long pm25=0;
long pm100=0;
char buf[50];
//const byte set = 2;
char verif = 0;
  
/* Parametre LED de verification */
#define LED_verte 6
#define LED_rouge 7

/* Parametre transistor */
#define pinTransistor 5           //Composant faisant office d'interrupteur

/* Parametre Horloge date et heure */
RTC_DS1307 RTC;                   //Classe RTC_DS1307

/* Parametre capteur Temperature et Humidite */
#define DHTPIN 8                  // what pin we're connected to
#define DHTTYPE DHT22             // DHT 22  (AM2302)
DHT dht(DHTPIN, DHTTYPE);

/* Parametre Sigfox */
#define RX 2
#define TX 3
SoftwareSerial sigfox(TX, RX);
uint8_t msg[4];
//unsigned long previousMillis = 0; //Nécessaire pour gérer la fréquence d'envoi
unsigned long interval = 10;        //Necessaire pour créer 10 boucles avant  d'envoyer un message
uint8_t delais_transmi = 0;         //Initialisation du compteur de boucle
float moy_pm10 =0 ;                 //Permet de calculer les moyennes
float moy_pm25 =0 ;


/***************************************************************/
/***************************SETUP*******************************/
/***************************************************************/

void setup() {
  Serial.begin(9600);                 //Ouverture du port série pour communication avec capteur PM
  sigfox.begin(9600);                 //Ouverture du port série pour communication modem Sigfox
  RTC.begin();                        //Demarrage de la librairie RTClib.h (date et heure)
  dht.begin();                        //Demarrage de la librairie dht (temperature et humidite)
Serial.print("Initialisation...");

  pinMode (LED_rouge, OUTPUT);        // 2 LED allumees : Batteries correctement branchees
  digitalWrite (LED_rouge, HIGH);
  flash_verte();                      
  delay(1000);
 
  /*parametrage du transistor */
  pinMode(pinTransistor, OUTPUT);         //Le transistor laisse passer le courant : capteur PM en mode fonctionnement
  digitalWrite(pinTransistor, HIGH);
  delay(1000);

  //Si RTC ne fonctionne pas
  if (! RTC.isrunning()) {
    flash_rouge ();                   // Flash rouge : Horloge mal branchee
    return;  
  }
  else {
    flash_verte();                    // Flash verte no1 : Horloge ok
  }

   /* Test du capteur PM */
  calculPM();                         // Flash rouge si capteur pas branche (cf fonction calculPM)
  delay(1000);
  
  if (verif == 2) {
    flash_verte ();                   // Flash verte no 2 : capteur PM ok
  }
  else { 
    flash_rouge ();                   // Flash rouge : capteur PM ne fonctionne pas
    return;
  }

  /* Ouverture du transistor */
  digitalWrite(pinTransistor, LOW);    //Le transistor ne laisse plus passer le courant : capteur PM ne consomme plus
  pinMode (pinTransistor, INPUT);
     
  /* Initialisation OK */
  digitalWrite (LED_rouge, LOW);
  pinMode (LED_rouge, INPUT);
  delay (2000);

  /* Test de la connection du module Sigfox */
  sigfox.write("AT$SF=0123cafe\r\n");           // Envoie du 1er message SigFox pour verifier la connection
}


/***************************************************************/
/*****************************LOOP******************************/
/***************************************************************/

void loop() {
    /* Appelle de la fonction de mise en veille de l'arduino */
    veilleArduino ();
   
    /* Appelle de la fonction de lecture du DHT22 (Temperature, Humidite) */
    temp_hum ();

    /* Appelle de la focntion de lecture du capteur PM */
    calculPM();
    
    /* Fermeture du transistor */
    digitalWrite(pinTransistor, LOW);    //Le transistor ne laisse plus passer le courant  
    pinMode (pinTransistor, INPUT);

    /* Appelle de la focntion d'envoi de message par Sigfox */
    message_sigfox ();
}


/***************************************************************/
/************************Fonctions******************************/
/***************************************************************/
/* watchdog interrupt */
ISR (WDT_vect) 
{
   wdt_disable();       // disable watchdog
}  


/* flash d'une led verte pour controle visuel */
void flash_verte ()
{
  pinMode (LED_verte, OUTPUT);        //Allumage de la LED
  digitalWrite (LED_verte, HIGH);
  delay (2000);                       //Attente de 2 secondes
  digitalWrite (LED_verte, LOW);      //Exctinction de la LED
  pinMode (LED_verte, INPUT);
  delay (2000);
}


/* flash d'une led rouge pour controle visuel */
void flash_rouge ()
{
  digitalWrite (LED_rouge, LOW);      //La LED rouge est toujours allumee (pour verifier que le courant passe correctement)
  pinMode (LED_rouge, INPUT);         //du coup il faut l'eteindre puis la rallumer pour signaler visuellement un probleme
  delay (2000);
  pinMode (LED_rouge, OUTPUT);
  digitalWrite (LED_rouge, HIGH);
  delay (2000);
}


/* Calcul des concentrations PM */
void calculPM() 
{
  /* Reveille du capteur PM */
  //pinMode (set, OUTPUT);
  //digitalWrite (set, HIGH);
  
  int count1 = 0;
  pm25 = 0;
  pm100 = 0;
  unsigned char c;
  unsigned char high;

  //Serial.println(s->available());
  if(Serial.available()!=0) {
    while (Serial.available()) {
      c = Serial.read();
      if(count1 > 15){
         break;
        }
        else if(count1 == 4 || count1 == 6 || count1 == 8 || count1 == 10 || count1 == 12 || count1 == 14) {
          high = c;       }
        else if(count1 == 5){
          pm25 = c; 
        }
        else if(count1 == 6){
          pm25 = c; 
        }
        else if(count1 == 13){ 
          pm25 = 256*high + c;
          verif = verif +1;        
        }
        else if(count1 == 15){
          pm100 = 256*high + c;
          verif = verif +1; 
        }
      count1++;
    }
    while(Serial.available()) Serial.read();
    delay(5000);    
  }
  else { flash_rouge();}
}


/* Recuperation des valeurs de temperature et d'humidite */
void temp_hum () 
{
  int h = dht.readHumidity();
  int t = dht.readTemperature();
}


/* Mise en veille de l'arduino */
void veilleArduino ()
{
  int i=1;                                            //initialisation d'un compteur
  while (i< 7){                                       //Endort l'arduino pendant 7 tours = 30 secondes
    /* Mise en stanby du capteur PM */
    //digitalWrite (set, LOW);
    //pinMode (set, INPUT);
    if (i==5 || i==6 ){
      /* Fermeture du transistor*/
      pinMode (pinTransistor, OUTPUT);
      digitalWrite (pinTransistor, HIGH);             //Le transistor laisse passe le courant
      delay(4000);
    }
    else {
      /* Ouverture du transistor */
      digitalWrite(pinTransistor, LOW);               //  Le transistor bloque le courant, le ventilateur n'est plus alimente
      pinMode (pinTransistor, INPUT);
    }
    
    /*Extinction de modules pour gagner en autonomie */
    ADCSRA = 0;                                       // disable ADC
    power_all_disable();                              // turn off all modules
    MCUSR = 0;                                        // clear various "reset" flags
    WDTCSR = bit (WDCE) | bit (WDE);                  // allow changes, disable reset 
    WDTCSR = bit (WDIE) | bit (WDP3) | bit (WDP0);    // set WDIE, and 8 seconds delay
    wdt_reset();  // pat the dog                      // set interrupt mode and an interval
    set_sleep_mode (SLEEP_MODE_PWR_DOWN);  
    noInterrupts ();                                  // timed sequence follows
    sleep_enable();
    
    /* turn off brown-out enable in software */
    MCUCR = bit (BODS) | bit (BODSE);                 // turn on brown-out enable select
    MCUCR = bit (BODS);                               // this must be done within 4 clock cycles of above
    interrupts ();                                    // guarantees next instruction executed
    sleep_cpu (); 
    // sleep within 3 clock cycles of above
    
    /* cancel sleep as a precaution*/
    sleep_disable();
    
    /* Reactive les peripheriques. */
    power_all_enable();
    
    /* incrementation du conteur pour la boucle */
    i=i+1;
  }
}


/* Fonction pour convertir un tableau d'octets en chaine hexadecimale */
char * convert(uint8_t * buffer, size_t len) {
  char * s = (char *) malloc(2 * len + 1);
  char * p = s;
  if( s ) {
    while( len-- ) {
      sprintf(p, "%02X", *buffer);
      p += 2;
      buffer++;
    }
  }
  return s;
}


/* Envoi d'un message par Sigfox */
void message_sigfox () 
{
Serial.println("");
Serial.print("delais_transmi= ");
Serial.println(delais_transmi);
Serial.print("interval= ");
Serial.println(interval);
  //unsigned long currentMillis = millis();
  if(delais_transmi > interval)                         //Si le temps ecoule est supérieur à 15 minutes
  { 
Serial.println("######boucle envoie####");
    moy_pm10 = moy_pm10 / (delais_transmi+1);           //Calcul de la moyenne des PM sur la période
    moy_pm25 = moy_pm25 / (delais_transmi+1);
    
    delais_transmi = 0;                                 //Reinitialise le compteur a zero
      
Serial.println("#1");   
    sigfox.println("\r\n");                             //Reveil du modem Sigfox
    //previousMillis = currentMillis; 

    msg[0] = (int) moy_pm10;
    msg[1] = (int) moy_pm25;
    msg[2] = dht.readHumidity();
    msg[3] = dht.readTemperature();
Serial.print("moyenne PM10= ");
Serial.println(msg[0]);
Serial.print("moyenne PM2.5= ");
Serial.println(msg[1]);
Serial.print("Humidite= ");
Serial.println(msg[2]);
Serial.print("temperature= ");
Serial.println(msg[3]);
Serial.println("#2");
    
    char * message = convert(msg, 4);                   //Conversion du tableau d'octets
    char * AT = (char * ) "AT$SF=";                     //Construction de la commande AT
    char * commande = (char *) malloc(strlen(message) + strlen(AT) + (size_t) 1);
Serial.println("#3");
    strcpy(commande, AT);
Serial.println("#4");
    strcat(commande, message);
Serial.println("#5");
    strcat(commande, "\r");
Serial.println(commande);                         //Affichage dans le moniteur du message envoyé
    sigfox.write(commande);                             //Envoi au modem de la commande
Serial.println("#6");
    free(commande);                                     //Nettoyage de la commande pour gain de mémoire
Serial.println("#7");
        
    if (sigfox.available()> 0)                        //Lecture de la réponse du modem Sigfox
    {
      Serial.println(sigfox.readStringUntil('\n'));
    }
Serial.println("#8"); 
    sigfox.println("AT$P=1\r");                         //Passage en mode sleep du modem Sigfox
    moy_pm10 = 0;
    moy_pm25 = 0;
Serial.println("#9"); /
    delay(2000);
  }
  else {
Serial.println("boucle moyenne");
    moy_pm10 = moy_pm10 + pm100;
    moy_pm25 = moy_pm25 + pm25;
    
    delais_transmi ++ ;
  }
}
