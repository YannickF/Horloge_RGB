#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>       
#include <RTClib.h>                   // Bibliothèque Adafruit
#include <BH1750.h>
#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>

#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <ESP8266WebServer.h>

#include "password.h"
#define NAME  "Horloge RGB"
#define VERSION   "1.8"
#define HOSTNAME "HorlogeRGB"
#include <TimeLib.h>
#include <NtpClientLib.h>
#define FASTLED_ESP8266_NODEMCU_PIN_ORDER
#include <FastLED.h>

// *******************************************************
// *              PARAMETRES UTILISATEUR                 *
// *******************************************************
// À commenter pour désactiver certaines fonctions
#define MAJ_OTA
#define DEBUG
#define CLIGNOTEMENT_SEPARATEURS                            // À commenter si on ne veut pas voir les séparateurs clignoter
#define AFFICHE_TEMPERATURE                                 // À commenter si on ne veut jamais voir la temperature
#define AFFICHE_HUMIDITE                                    // À commenter si on ne veut jamais voir l'humidite
#define AFFICHE_TEMP_DIXIEME                                // À commenter pour ne pas avoir les dixièmes de degré
//#define AFFICHE_HELLO                                       // À commenter pour ne pas avoir le HELLOo au démarrage
#define CHANGEMENT_HEURE_SAISONNIER                         // À commenter pour ne pas avoir de mise à l'heure à 2h et 3h chaque jour
#define BIPS                                                // À commenter pour ne pas avoir de bip chaque heure

#define BAUD                    115200
#define DELAI_AFFICHAGE_TEMP_HUM 46                         // temps (en s) entre 2 affichages de la température et de l'humidité
#define TEMPS_AFFICHAGE         2500                        // durée (en ms) d'affichage de la température et de l'humidité
#define DELAI_HELLO             2500                        // durée (en ms) d'affichage du message HELLO
#define CALIBRAGE_TEMPERATURE   -1                          // décalage du capteur DHT par rapport à la réalité
#define LUM_AMBIANTE_MIN        0                           // valeur mini de luminosité ambiante
#define LUM_AMBIANTE_MAX        1000                        // Valeur maxi de luuminsosité ambiante (54612 pour le capteur BH1750 mais ça correspond à une luminosté énorme
#define COEFF_LUM               6                           // coefficient multiplicateur de la luminosité ambiante pour calculer celle du ruban

#define NTP_SERVER              "fr.pool.ntp.org"
#define NTP_DELAI               3600*2                      // temps en secondes entre 2 interrogations du serveur NTP
#define Hsaison                 2                           // heures du changement d'heure saisonnier (+1 auto pour l'autre saison)
#define Msaison                 0                           // minutes du changement d'heure saisonnier
#define Ssaison                 8                           // secondes du changement d'heure saisonnier
#define TEMP_HSV_ROUGE          30                          // température qui donnera du rouge (au dessus également)
#define TEMP_HSV_BLEU           5                           // température qui donnera du bleu (en dessous également)
#define DECALAGE_TEMP_HSV       -5                          // coefficient additif pour obtenir la bonne couleur
#define DUREE_BIP_00            250                         // durée en ms du bip aux heures pleines
#define DUREE_BIP_30            50                          // durée en ms du bip aux 1/2 heures
#define DUREE_BIP_MESSAGE       2000                         // durée en ms du bip en début et fin de message texte défilant         
// *******************************************************
// *              AFFICHEUR 7 SEGMENTS                   *
// *******************************************************
//      c             Le ruban de LED WS2812B est cablé de a vers g avant d'aller 
//     ---            à l'afficheur 7 segments suivant
//   b|   | d         les points séparateurs sont en dernier sur le ruban
//     -a-
//   g|   | e
//     ---
//      f
// tableau contenant les segments à allumer. Chaque élements est un octet au format (binaire) xabcdefg  (x = 0 quel que soit le caractère à afficher)
// pour les autres si c'est 1 on allume le segment, sinon on laisse éteint
const char chiffreslettres[70] = {
                                0b0111111,                // 0        position 0 dans le tableau
                                0b0001100,                // 1
                                0b1011011,                // 2
                                0b1011110,                // 3
                                0b1101100,                // 4
                                0b1110110,                // 5
                                0b1110111,                // 6
                                0b0011100,                // 7
                                0b1111111,                // 8
                                0b1111110,                // 9
                                0b1111101,                // A ou 8    position 10 dans le tableau
                                0b1111111,                // B
                                0b0110011,                // C
                                0b0011111,                // D
                                0b1110011,                // E
                                0b1110001,                // F ou F
                                0b0110111,                // G
                                0b1101101,                // H
                                0b0100001,                // I
                                0b0001111,                // J
                                0b1101011,                // K
                                0b0100011,                // L
                                0b1010101,                // M
                                0b1101101,                // N
                                0b0111111,                // O ou 0
                                0b1111001,                // P ou p
                                0b0111011,                // Q
                                0b1111011,                // R
                                0b1110110,                // S ou 5
                                0b0110001,                // T
                                0b0101111,                // U
                                0b0101110,                // V
                                0b1101010,                // W
                                0b1010010,                // X
                                0b1101110,                // Y ou y
                                0b1011011,                // Z ou 2
                                0b1000110,                // a       position 36 dans le tableau
                                0b1100111,                // b
                                0b1000011,                // c
                                0b1001111,                // d
                                0b0000011,                // e
                                0b1110001,                // f ou F
                                0b1010011,                // g
                                0b1100101,                // h
                                0b0000001,                // i
                                0b0001110,                // j
                                0b1110010,                // k
                                0b0100001,                // l
                                0b0010101,                // m                                
                                0b1000101,                // n
                                0b1000111,                // o ou ROND BAS
                                0b1111001,                // p ou P
                                0b1111100,                // q
                                0b1000001,                // r
                                0b1000010,                // s
                                0b1100011,                // t
                                0b0000111,                // u
                                0b0000110,                // v
                                0b0101010,                // w 
                                0b1000010,                // x ou y ou EGAL
                                0b1101110,                // y ou Y
                                0b1000010,                // Z ou x ou EGAL
                                0b0000000,                // ESPACE     position 62
                                0b0101000,                // "
                                0b0100000,                // '
                                0b0110011,                // ( ou [
                                0b0011110,                // ) ou ]
                                0b1000000,                // -
                                0b0000010,                // _        
                                0b1111000                 // °

                                
};
//position des caractères spéciaux dans le tableau précédent
#define H                       17
#define ROND_BAS                50
#define EGAL                    61
#define ESPACE                  62
#define DOUBLEQUOTE             63
#define SIMPLEQUOTE             64
#define PARENTHESE_OUVRE        65
#define PARENTHESE_FERME        66
#define MOINS                   67
#define UNDERSCORE              68
#define DEGRE                   69






// *******************************************************
// *              CONSTANTES IMPORTANTES                 *
// *******************************************************
#define NB_LED_PAR_SEGMENT      4                         // nombre de LED par segment
#define NB_LED_PAR_AFFICHEUR    7 * NB_LED_PAR_SEGMENT    // nombre de LED par afficheur 7 segments
#define NB_AFFICHEUR            6                         //  heures:minutes:secondes
#define NB_LED_SEPARATEUR       2                         // 1 led par point des séparateurs
#define NUM_LEDS                (NB_AFFICHEUR * NB_LED_PAR_AFFICHEUR) + NB_LED_SEPARATEUR * 2    // nombre total de LED sur le ruban
#define NB_LED_PAR_ETAPE        21                        // nombre de LED pour afficher la progression du setup() sur les LED

// position des differents chiffres le long du ruban (0= début du ruban)
#define UNITE_SECONDES          0                         // position de la première LED
#define DIZAINE_SECONDES        UNITE_SECONDES + 7 * NB_LED_PAR_SEGMENT
#define SEPARATEUR_MS           DIZAINE_SECONDES + 7 * NB_LED_PAR_SEGMENT
#define UNITE_MINUTES           SEPARATEUR_MS + NB_LED_SEPARATEUR
#define DIZAINE_MINUTES         UNITE_MINUTES + 7 * NB_LED_PAR_SEGMENT
#define SEPARATEUR_HM           DIZAINE_MINUTES + 7 * NB_LED_PAR_SEGMENT
#define UNITE_HEURES            SEPARATEUR_HM + NB_LED_SEPARATEUR
#define DIZAINE_HEURES          UNITE_HEURES + 7 * NB_LED_PAR_SEGMENT

#define LUM_MINI                32                        // luminosité mini du bandeau LED
#define LUM_MAXI                255                       // luminosité maxi du bandeau LED
#define HUE_MINI                0                         // couleur hue min (rouge)
#define HUE_MAXI                160                       // couleur hue max (bleu)

#define NTP_TIMEOUT             1000               
#define NTP_OFFSET              1                         // fuseau horaire : pour la France UTC+1 
#define NTP_DAYLIGHT            true                      //changement heure été / heure hiver
#define DELAI_WIFI              10000                     // delai en millisecondes avant de considérer que la connexion Wifi n'a pas été établie

#define ECART_MAX               20                         // Ecart maximum (en s) entre les 2 horloges NTP et RTC (NTP étant la plus fiable)

#define RETOURLIGNE             true
#define NO_RETOURLIGNE          false
#define AFFICHE_DATE            true
#define NO_AFFICHE_DATE         false
#define AFFICHE_JOUR_SEMAINE    true
#define NO_AFFICHE_JOUR_SEMAINE false

#define DHT_DATA_PIN            D5
#define DHT_TYPE                DHT22 

#define SDA_PIN                 D1
#define SCL_PIN                 D2
#define BH1750_I2C_ADDRESS      0x23                        // peut être 0x5C si on met la broche ADDR à VCC

#define LED_TYPE                WS2812B

#define LED_DATA_PIN            6                           // broche de controle des LED  D4 = 2 sur nodeMCU
#define MAX_LED_POWER           20000                       // puissance Maxi en mW pour le ruban LED

#define PORT                    80                          // port HTTP du serveur web

#define HISTO_TEMP_DELAI        5                           // Temps (en min) entre 2 mémorisation des températures

#define BUZZER                  D8                          // broche pour le buzzer

// *******************************************************
// *                 VARIABLES GLOBALES                  *
// *******************************************************
const char BACKSPACE                  = 0x08;               // caractère d'effacement sur le port série (pas toujours gérer par les terminaux)
bool affiche_humidite                 = false;              // pour afficher l'humidité une fois sur 2.
bool clignote                         = true;               // pour gérer le clignotement des séparateurs
bool boot                             = true;               // pour détecter le 1er démarrage de l'horloge
int timerTH                           = 0;                  // pour compter le délai entre 2 affichages de la température/humidité
DHT DHTsensor(DHT_DATA_PIN, DHT_TYPE);                      // Creation d'un objet DHT pour utiliser le capteur
float humidite                        = 50;                 // pour stocker la valeur du taux d'humidité
float temperature                     = 20;                 // pour stocker la valeur de la température
BH1750 BH1750sensor(BH1750_I2C_ADDRESS);                    // création d'un objet BH1750 pour utiliser le capteur
int luminosite                        = 5000;               // pour stocker la valeur de la luminosité
CRGB leds[NUM_LEDS];                                        // création d'un objet pour gérer le ruban de LED
ESP8266WebServer server(PORT);                              // création d'un objet pour le serveur Web
float histo_temp[31]= {[0]=-300};                            // stockage de l'historique des températures, toutes les 5 minutes (-300 indique que le tableau n'est pas initialisé)

char position_digit[8]={UNITE_SECONDES,DIZAINE_SECONDES,UNITE_MINUTES,DIZAINE_MINUTES,UNITE_HEURES,DIZAINE_HEURES,SEPARATEUR_MS,SEPARATEUR_HM};

#define DS3231                                              // #define DS3231 or DS1307   Voir plus bas (variables globales)
#ifdef DS1307
  RTC_DS1307 rtc;
  #define SQUAREWAVE DS1307_SquareWave1HZ
//  #define SQUAREWAVE DS1307_SquareWave4kHz
//  #define SQUAREWAVE DS1307_SquareWave8kHz
//  #define SQUAREWAVE DS1307_SquareWave32kHz
#endif
#ifdef DS3231
  RTC_DS3231 rtc; 
  #define SQUAREWAVE DS3231_SquareWave1Hz
  //#define SQUAREWAVE DS3231_SquareWave1kHz
  //#define SQUAREWAVE DS3231_SquareWave4kHz
  //#define SQUAREWAVE DS3231_SquareWave8kHz
#endif

DateTime dateheure_rtc;
WiFiClient client;
const char* ssid                      = STASSID;
const char* password                  = STAPSK;
bool wifi                             = false;              //true si le wifi est connecté, false sinon
bool ntp                              = false;              //true si l'heure est récupérée par NTP
bool dsrtc                            = false;              //true si l'horloge RTC est détectée
bool dht                              = false;              //true si le capteur DHT température/humidité est détecté
bool bh                               = false;              //true si le capteur BH1750 luminosité est détecté
unsigned char etape                   = 0;                  //suivi de la progression des étapes de configuration (pas indispensable d'être globale, mais c'est plus simple)
int Hheure=0,Mminute=0,Sseconde       = 0;
int old_Hheure=-1, old_Mminute=-1, old_Sseconde=-1;
int led_luminosite                    = 32;                 //luminosité par défaut du ruban de LED [0..255]]
int led_temperature                   = 0;                  //couleur par défaut en fonction de la temperature sur l'échelle Hue [0..255]

// pour stocker les couleurs des digits
CRGB couleurDH                        = CRGB::Green;
CRGB couleurUH                        = CRGB::Green;
CRGB couleurDM                        = CRGB::Green;
CRGB couleurUM                        = CRGB::Green;
CRGB couleurDS                        = CRGB::Green;
CRGB couleurUS                        = CRGB::Green;
CRGB couleurSEPHM                     = CRGB::Green;
CRGB couleurSEPMS                     = CRGB::Green;


// *******************************************************
// *                       FONCTIONS                     *
// *******************************************************

void  affichage_histo(void) {
  Serial.println();
  Serial.println("Tableau des mesures :");

  for (int i=0;i<=30;i++) {
    Serial.print("[");Serial.print(i);Serial.print("] = ");Serial.println(histo_temp[i]);

  }
}
int my_asc(int lettre) {    // retourne la position d'une lettre dans le tableau des caractères
  if (lettre >= 48 && lettre <= 57) {return lettre-48;}             // chiffres
  else if (lettre >= 65 && lettre <= 90) {return lettre-55;}        // majuscules
  else if (lettre >= 97 && lettre <= 122) {return lettre-61;}       // minuscules
  else if (lettre==95) {return UNDERSCORE;}
  else if (lettre==45) {return MOINS;}
  else if (lettre==61) {return EGAL;}
  else if (lettre==32) {return ESPACE;}
  else if (lettre==42 || lettre == 194 || lettre == 176) {return DEGRE;}         // asterisque pour remplacer le degré
  else if (lettre== 34) {return DOUBLEQUOTE;}
  else if (lettre==39) {return SIMPLEQUOTE;}
  else if (lettre==40 || lettre==91) {return PARENTHESE_OUVRE;}
  else if (lettre==41 || lettre==93) {return PARENTHESE_FERME;}
  else {return UNDERSCORE;}   // caractère non affichable
}
void lecture_DHT(bool retour_ligne=true) {
  float newH = DHTsensor.readHumidity();
  float newT = DHTsensor.readTemperature();
  if (isnan(newT)) {
    Serial.println(" Erreur de lecture Temp. DHT !");
    dht=false;
  } else {
    temperature = newT + CALIBRAGE_TEMPERATURE;
    Serial.print("Température DHT = ");
    Serial.print(temperature,1);
    Serial.print(" °C ");
  }
  if (isnan(newH)) {
    Serial.println(" Erreur de lecture Humid. DHT !");
    dht=false;
  } else {
    humidite = newH;
    Serial.print("Humidité DHT = ");
    Serial.print(humidite,0);
    Serial.print(" % ");
  }
  if (!isnan(newT) && !isnan(newH)) {dht=true;}
  if (retour_ligne) {Serial.println();}

}
void lecture_BH1750(bool retour_ligne) {
  BH1750sensor.begin(BH1750::ONE_TIME_LOW_RES_MODE);
  delay(200);
  luminosite =(int) BH1750sensor.readLightLevel();
  Serial.print(" Luminosité = ");
  Serial.print(luminosite,1);
  Serial.print(" ");
  if (retour_ligne) {Serial.println();}
}
long remapping(long x, long in_min, long in_max, long out_min, long out_max) {
  long reponse = (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
  if (reponse < out_min) reponse=out_min;
  if (reponse > out_max) reponse=out_max;
  return reponse;
}
int calcul_couleur_temp(float temperature) {
  int valeur = remapping((int) ((temperature+DECALAGE_TEMP_HSV)*10), TEMP_HSV_ROUGE*10,TEMP_HSV_BLEU*10,HUE_MINI,HUE_MAXI);  // conversion de la température en couleur dans l'espace HSV Rainbow (le *10 sert à prendre en compte la valeur des dixièmes de °C)
  if (valeur<HUE_MINI) {valeur=HUE_MINI;}
  if (valeur>HUE_MAXI) {valeur=HUE_MAXI;}
  return valeur;
}

void affichage_ruban_caractere(unsigned char position, unsigned char caractere, CRGB couleur) {
  // position = n° de l'afficheur 7 segments 
  int debut =position;  //position de la 1ère LED (au niveau du ruban) dans l'afficheur
  for (int segment=0;segment<7; segment++) {   // pour chaque segment
    char allumesegment =  (chiffreslettres[caractere]<<segment) & 0b01000000;  // on recupere juste le bit du segment concerné
    if (allumesegment) {
      for (char i=0; i< NB_LED_PAR_SEGMENT; i++) {
        leds[debut+ segment * NB_LED_PAR_SEGMENT + i] = couleur;
        //Serial.print("Segment:");Serial.print(segment);Serial.print(" Led ON:");Serial.println(debut+ segment * NB_LED_PAR_SEGMENT + i); 
      }
    } else {
      for (char i=0; i< NB_LED_PAR_SEGMENT; i++) {
        leds[debut+ segment * NB_LED_PAR_SEGMENT + i] = CRGB::Black;
        //Serial.print("Segment:");Serial.print(segment);Serial.print(" Led OFF:");Serial.println(debut+ segment * NB_LED_PAR_SEGMENT + i);
      }

    }
  }
  
}
void affichage_ruban_secondes(unsigned char valeur, CRGB couleur10, CRGB couleur1) {
  int dizaine = valeur / 10;
  int unite = valeur % 10;
  //Serial.println();
  //Serial.print("U:");Serial.print(unite);Serial.print(" D:");Serial.println(dizaine);
  affichage_ruban_caractere(UNITE_SECONDES, unite, couleur1);
  affichage_ruban_caractere(DIZAINE_SECONDES, dizaine, couleur10);
}
void affichage_ruban_minutes(unsigned char valeur, CRGB couleur10, CRGB couleur1) {
  int dizaine = valeur / 10;
  int unite = valeur % 10;
  affichage_ruban_caractere(UNITE_MINUTES, unite, couleur1);
  affichage_ruban_caractere(DIZAINE_MINUTES, dizaine, couleur10);
  
}
void affichage_ruban_heures(unsigned char valeur, CRGB couleur10, CRGB couleur1) {
  int dizaine = valeur / 10;
  int unite = valeur % 10;
  affichage_ruban_caractere(UNITE_HEURES, unite, couleur1);
  affichage_ruban_caractere(DIZAINE_HEURES, dizaine, couleur10);
  
}
void affichage_ruban_separateur(int position,CRGB couleur) {
  for (int i = 0; i<NB_LED_SEPARATEUR ; i++) {
    leds[position+i] = couleur;
  }
}
void affichage_ruban_point(int position,CRGB couleur) {
  leds[position] = CRGB::Black;
  leds[position+1] = couleur;
}
  void affiche_heure(DateTime date, bool jour_semaine=true, bool affiche_date=true, bool retour_ligne=true) {
    if ( date.hour()<10 ) Serial.print("0");
    Serial.print(date.hour(), DEC);
    Serial.print(':');
    if ( date.minute()<10 ) Serial.print("0");
    Serial.print(date.minute(), DEC);
    Serial.print(':');
    if ( date.second()<10 ) Serial.print("0");
    Serial.print(date.second(), DEC);
    Serial.print(" ");

    if (affiche_date) {
      if ( date.day()<10 ) Serial.print("0");
      Serial.print(date.day(), DEC);
      Serial.print('/');
      if ( date.month()<10 ) Serial.print("0");
      Serial.print(date.month(), DEC);
      Serial.print('/');
      Serial.print(date.year(), DEC);
      if (jour_semaine) {
        char daysOfTheWeek[7][12] = {"Dimanche", "Lundi", "Mardi", "Mercredi", "Jeudi", "Vendredi", "Samedi"};
        Serial.print(" (");
        Serial.print(daysOfTheWeek[date.dayOfTheWeek()]);
        Serial.print(") ");
      }
    }
    if (retour_ligne) {Serial.println();}
}
void allumage_led_progression(int num_debut, int nombre_led, CRGB couleur) {
  for (int i=num_debut; i < num_debut+nombre_led ;i++){
    yield();
    leds[i] = couleur;
    FastLED.show();
    yield();
    delay(10);
  }
  
}
void affichage_ruban_temperature(void) {
  // AFFICHAGE
  affichage_ruban_caractere(DIZAINE_HEURES,ESPACE,couleurDH);
  if (temperature <0 ) 
  {
    affichage_ruban_caractere(UNITE_HEURES,MOINS,couleurUH);
  }else {
    affichage_ruban_caractere(UNITE_HEURES,ESPACE,couleurUH);
  }
  affichage_ruban_separateur(SEPARATEUR_HM,CRGB::Black);
    
  #ifndef AFFICHE_TEMP_DIXIEME
  signed int temp= round(abs(temperature));
  int dizaine = temp / 10;
  int unite = temp % 10;
  affichage_ruban_caractere(DIZAINE_MINUTES, dizaine, couleurDM);
  affichage_ruban_caractere(UNITE_MINUTES, unite, couleurUM);
  affichage_ruban_separateur(SEPARATEUR_MS,CRGB::Black);
  affichage_ruban_caractere(DIZAINE_SECONDES,DEGRE,couleurDS);
  affichage_ruban_caractere(UNITE_SECONDES,C,couleurUS);
  #else
  signed int temp = (signed int) abs(temperature * 10);                    // on multiplie par 10 pour avoir un entier à partir d'un nombre décimal
  int dizaine = temp / 100;
  int unite = (temp -dizaine * 100) / 10;
  int dixieme = temp % 10;
  affichage_ruban_caractere(DIZAINE_MINUTES, dizaine, couleurDM);
  affichage_ruban_caractere(UNITE_MINUTES, unite, couleurUM);
  affichage_ruban_point(SEPARATEUR_MS,couleurSEPMS);
  affichage_ruban_caractere(DIZAINE_SECONDES,dixieme,couleurDS);
  affichage_ruban_caractere(UNITE_SECONDES,DEGRE,couleurUS);
  #endif
}
void affichage_ruban_humidite(void) {
  int hum= (int) humidite;
  int dizaine = hum / 10;
  int unite = hum % 10;
  affichage_ruban_caractere(DIZAINE_HEURES,H,couleurDH);
  affichage_ruban_caractere(UNITE_HEURES,EGAL,couleurUH);
  affichage_ruban_separateur(SEPARATEUR_HM,CRGB::Black);
  affichage_ruban_caractere(DIZAINE_MINUTES, dizaine, couleurDM);
  affichage_ruban_caractere(UNITE_MINUTES, unite, couleurUM);
  affichage_ruban_separateur(SEPARATEUR_MS,CRGB::Black);
  affichage_ruban_caractere(DIZAINE_SECONDES,DEGRE,couleurDS);
  affichage_ruban_caractere(UNITE_SECONDES,ROND_BAS,couleurUS);
}
void affichage_ruban_message_fixe(int position,char *message,int duree,CRGB *couleur) {
  bool EOT = false;    // fin du texte ?
  if (position <0) {position=0;}
  for (int i=position;i >= 0; i--) {
    if (i>=NB_AFFICHEUR) {
      // on n'affiche rien mais on analyse un par un les caractères du message
      if (message[position-i] == 0) {EOT=true;}
    }
    if ((i< NB_AFFICHEUR)&& !EOT) {
      if (message[position-i] == 0) {
        EOT=true;
      } else {
      affichage_ruban_caractere(position_digit[i],my_asc(message[position-i]), couleur[i]);
      }
    }
    if ((i< NB_AFFICHEUR) && EOT) {
      affichage_ruban_caractere(position_digit[i],ESPACE, CRGB::Black);
    }
  }
  
  affichage_ruban_separateur(SEPARATEUR_HM,CRGB::Black);
  affichage_ruban_separateur(SEPARATEUR_MS,CRGB::Black);
  FastLED.show();
  delay(duree);
  FastLED.clear();
}
void affichage_ruban_message_scroll(char *message, int duree, CRGB *couleur) {
  int i=0;
  while (message[i] != 0) {
    affichage_ruban_message_fixe(i,message,duree,couleur);
    i++;
  }
  for (int j=0; j<NB_AFFICHEUR;j++) {
    affichage_ruban_message_fixe(i+j,message,duree,couleur);
  }
}
void handle_NotFound(){
  server.send(404, "text/plain", "Erreur 404 : Page inexistante !");
}
void bip(int duree) {
  digitalWrite(BUZZER,HIGH);
  delay(duree);
  digitalWrite(BUZZER,LOW);
}
void handle_OnConnect() {
  int delai=500;
  CRGB couleurs[NB_AFFICHEUR]={CRGB::Red,CRGB::Red,CRGB::Red,CRGB::Red,CRGB::Red,CRGB::Red};
  Serial.println();
  Serial.println("Requete HTTP reçue");
  if (!server.authenticate(www_username, www_password)) {
      return server.requestAuthentication();
  }
  if (server.hasArg("delai")) {
    delai=server.arg("delai").toInt();
    if (delai>2000) {delai=2000;}
  }
  if (server.hasArg("message")) {
    server.send(200, "text/html", "Message = "+ server.arg("message") +"\nDelai = " + delai+ "\nOK"); 
    char chaine[128]="Bonjour";
    server.arg("message").toCharArray(chaine,128,0);
    bip(DUREE_BIP_MESSAGE);
    affichage_ruban_message_scroll(chaine,delai,couleurs);
    bip(DUREE_BIP_MESSAGE);
  } else {
    server.send(200, "text/html", "Sans Arguments");
  }
}


// *******************************************************
// *                          SETUP                      *
// *******************************************************,
void setup() {
  Serial.begin(BAUD);
  Serial.println();
  Serial.print(NAME);Serial.print(" V");Serial.println(VERSION);
    Serial.println("Configuration :");
  pinMode(LED_BUILTIN, OUTPUT);                     // broche LED_BUILTIN configurée en sortie
  pinMode(BUZZER,OUTPUT);                           // broche BUZZER configurée en sortie
  // Ruban LED 
  Serial.print(etape+1);
  Serial.print("-> Configuration du ruban ");
  if (etape %2) {digitalWrite(LED_BUILTIN, HIGH);} else {digitalWrite(LED_BUILTIN, LOW);}
  FastLED.addLeds<NEOPIXEL, LED_DATA_PIN>(leds, NUM_LEDS);
  FastLED.setMaxPowerInMilliWatts(MAX_LED_POWER);
  #ifdef AFFICHE_HELLO
  CRGB tab_couleur[NB_AFFICHEUR]={CRGB::Red,CRGB::OrangeRed,CRGB::Yellow,CRGB::Green,CRGB::Cyan,CRGB::Purple};
  //affichage_ruban_message_fixe(9,"COUcou",DELAI_HELLO,tab_couleur);
  affichage_ruban_message_scroll("Bonjour YAnnick",500,tab_couleur);
  //affichage_ruban_message_scroll("COUcou",500,tab_couleur);
  #endif
  //FastLED.setBrightness(led_luminosite);
  allumage_led_progression( etape * NB_LED_PAR_ETAPE, NB_LED_PAR_ETAPE , CRGB::Red);
  Serial.print(NUM_LEDS);Serial.print(" LEDs : ");
  Serial.println("OK");
  allumage_led_progression( etape * NB_LED_PAR_ETAPE, NB_LED_PAR_ETAPE , CRGB::Green);
  etape++;
   

  // Connexion Wifi
  Serial.print(etape+1);
  Serial.print("-> Connexion Wifi : ");
  if (etape %2) {digitalWrite(LED_BUILTIN, HIGH);} else {digitalWrite(LED_BUILTIN, LOW);}
  //allumage_led_progression( etape * NB_LED_PAR_ETAPE, NB_LED_PAR_ETAPE , CRGB::Yellow);
  Serial.print(ssid);
  Serial.print(" ");
  unsigned long timeout=millis();
  yield();
  WiFi.persistent(false);
  WiFi.mode(WIFI_STA);
  WiFi.hostname(HOSTNAME);
  WiFi.begin(ssid, password);
  int i=2;
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    allumage_led_progression( etape * NB_LED_PAR_ETAPE , i , CRGB::Yellow);
    allumage_led_progression( etape * NB_LED_PAR_ETAPE +i, 2 , CRGB::Red);
    i=i+2;
    if (millis()-timeout > DELAI_WIFI ) {
        Serial.println("  -> Connexion impossible au Wifi");
        wifi =false;
        break;
    } else {
         wifi = true ;
    }
  }
  if (wifi) {
    Serial.print(" -> WiFi connecté");
    server.on("/", HTTP_GET, handle_OnConnect);
    server.onNotFound(handle_NotFound);
    server.begin();
    Serial.print(" -> Serveur Web démarré");
    Serial.print(" -> Adresse IP: ");
    Serial.println(WiFi.localIP());  
    allumage_led_progression( etape * NB_LED_PAR_ETAPE, NB_LED_PAR_ETAPE , CRGB::Green);
  } else {
    allumage_led_progression( etape * NB_LED_PAR_ETAPE, NB_LED_PAR_ETAPE , CRGB::Red);
  }
  etape++;
  yield(); 

  // Paramètres OTA
  #ifdef MAJ_OTA
  if (wifi) {
    Serial.print(etape+1);
    Serial.println("-> Activation OTA");
    if (etape %2) {digitalWrite(LED_BUILTIN, HIGH);} else {digitalWrite(LED_BUILTIN, LOW);}
    allumage_led_progression( etape * NB_LED_PAR_ETAPE, NB_LED_PAR_ETAPE , CRGB::Yellow);
    // Port defaults to 8266
    // ArduinoOTA.setPort(8266);
    // Hostname defaults to esp8266-[ChipID]
    ArduinoOTA.setHostname(HOSTNAME);
    // No authentication by default
    ArduinoOTA.setPassword(PASSWORD);
    // Password can be set with it's md5 value as well
    // MD5(admin) = 21232f297a57a5a743894a0e4a801fc3
    // ArduinoOTA.setPasswordHash("21232f297a57a5a743894a0e4a801fc3");
    ArduinoOTA.onStart([]() {
      String type;
      if (ArduinoOTA.getCommand() == U_FLASH) {
        type = "sketch";
      } else { // U_FS
        type = "filesystem";
      }

      // NOTE: if updating FS this would be the place to unmount FS using FS.end()
      Serial.println();
      Serial.println("Mise à Jour " + type);
    });
    ArduinoOTA.onEnd([]() {
      Serial.println("\nFini !");
    });
    ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
      Serial.printf("Progression: %u%%\r", (progress / (total / 100)));
    });
    ArduinoOTA.onError([](ota_error_t error) {
      Serial.printf("Erreur[%u]: ", error);
      if (error == OTA_AUTH_ERROR) {
        Serial.println("Auth Failed");
      } else if (error == OTA_BEGIN_ERROR) {
        Serial.println("Begin Failed");
      } else if (error == OTA_CONNECT_ERROR) {
        Serial.println("Connect Failed");
      } else if (error == OTA_RECEIVE_ERROR) {
        Serial.println("Receive Failed");
      } else if (error == OTA_END_ERROR) {
        Serial.println("End Failed");
      }
    });
    yield();
    ArduinoOTA.begin();
    allumage_led_progression(etape * NB_LED_PAR_ETAPE, NB_LED_PAR_ETAPE , CRGB::Green);
    etape++;
  }
  #endif
  yield();

  // connexion au serveur NTP
  Serial.print(etape+1);
  Serial.print("-> Récupération de l'heure via NTP : ");
  if (etape %2) {digitalWrite(LED_BUILTIN, HIGH);} else {digitalWrite(LED_BUILTIN, LOW);}
  allumage_led_progression( etape * NB_LED_PAR_ETAPE, NB_LED_PAR_ETAPE , CRGB::Yellow);
  if(wifi) {
    Serial.print("-> Connexion au serveur NTP : ");
    NTP.onNTPSyncEvent ([](NTPSyncEvent_t error) {
        if (error) {
            if (error == requestSent) 
                Serial.print(" Requête NTP envoyée ");
            else if (error == noResponse)
                Serial.println ("-> Pas de réponse NTP !");
            else if (error == invalidAddress)
                Serial.println ("-> Adresse serveur NTP invalide !");
        } else {
            Serial.print ("-> Heure NTP OK : ");
            ntp=true;
            Serial.println (NTP.getTimeDateString (NTP.getLastNTPSync ()));
        }
    });
    NTP.setInterval(NTP_DELAI);
    NTP.setNTPTimeout (NTP_TIMEOUT);
    NTP.begin(NTP_SERVER, NTP_OFFSET, NTP_DAYLIGHT);          //l'ESP8266 va régulièrement se mettre à l'heure
    yield();
    
  } else {
    Serial.println(" Impossible, pas de connexion WiFi !");
  }
  delay(750);
  if (ntp) {
    allumage_led_progression( etape * NB_LED_PAR_ETAPE, NB_LED_PAR_ETAPE , CRGB::Green);
  }else {
    allumage_led_progression( etape * NB_LED_PAR_ETAPE, NB_LED_PAR_ETAPE , CRGB::Red);
  }
  etape++;
  yield();
  

  // Détection du capteur DHT
  Serial.print(etape+1);
  Serial.print("-> Capteur DHT :");
  if (etape %2) {digitalWrite(LED_BUILTIN, HIGH);} else {digitalWrite(LED_BUILTIN, LOW);}
  allumage_led_progression( etape * NB_LED_PAR_ETAPE, NB_LED_PAR_ETAPE , CRGB::Yellow);
  DHTsensor.begin();  // paramètrage du capteur sur la broche choisie
  delay(1500);
  float t = DHTsensor.readTemperature();
  if (isnan(t)) {
    Serial.println(F(" Erreur"));
    allumage_led_progression( etape * NB_LED_PAR_ETAPE, NB_LED_PAR_ETAPE , CRGB::Red);
  }
  else {
    Serial.println(" OK");
    allumage_led_progression( etape * NB_LED_PAR_ETAPE, NB_LED_PAR_ETAPE , CRGB::Green);
    dht=true;
  }
  etape++;
  yield();

  // Paramètrage du bus i2c
  Wire.begin(SDA_PIN, SCL_PIN);
  
   
  //Récupération de l'heure RTC sur i2c
  Serial.print(etape+1);
  allumage_led_progression( etape * NB_LED_PAR_ETAPE, NB_LED_PAR_ETAPE , CRGB::Yellow);
  if (etape %2) {digitalWrite(LED_BUILTIN, HIGH);} else {digitalWrite(LED_BUILTIN, LOW);}
  Serial.print("-> Récupération de l'heure via RTC : ");
  rtc.begin();
  #ifdef DS1307
  if (!rtc.isrunning()) {
  #endif
  #ifdef DS3231
  dsrtc = rtc.begin();
  if (!dsrtc) {
  #endif
    dsrtc= false;
    Serial.println("Horloge RTC non détectée, arrêtée, ou pile HS !");
    allumage_led_progression( etape * NB_LED_PAR_ETAPE, NB_LED_PAR_ETAPE , CRGB::Red);
  } else {
    dsrtc=true;
    allumage_led_progression( etape * NB_LED_PAR_ETAPE, NB_LED_PAR_ETAPE , CRGB::Green);
    dateheure_rtc = rtc.now();
    affiche_heure(dateheure_rtc,AFFICHE_JOUR_SEMAINE, AFFICHE_DATE, RETOURLIGNE);
    rtc.writeSqwPinMode(SQUAREWAVE);      // on active la sortie signal carré
    //rtc.writeSqwPinMode(DS1307_ON);
  }
  etape++;
  yield();
  Serial.print(etape+1);
  Serial.print("-> Synchronisation des 2 horloges : ");
  if (etape %2) {digitalWrite(LED_BUILTIN, HIGH);} else {digitalWrite(LED_BUILTIN, LOW);}
  allumage_led_progression( etape * NB_LED_PAR_ETAPE, NB_LED_PAR_ETAPE , CRGB::Yellow);
  if (dsrtc && ntp) {
    //comparaison des 2 horloges RTC et NTP
    Serial.println();
    Serial.print("   -> Horloge RTC = ");
    Serial.println(dateheure_rtc.unixtime());
    Serial.print("   -> Horloge NTP = ");
    Serial.println(now());
    signed long ecart = dateheure_rtc.unixtime() - now();
    ecart = abs(ecart);
    Serial.print("   -> Ecart = ");
    Serial.println(ecart);
    if (ecart > ECART_MAX) {      //RTC n'est pas à l'heure, on la règle sur l'heure NTP
      Serial.println("   -> Les heures NTP et RTC sont trop différentes.");
      Serial.print("     -> L'horloge RTC va être ajustée à : "); affiche_heure(now(),AFFICHE_JOUR_SEMAINE, AFFICHE_DATE, RETOURLIGNE);
      rtc.adjust( DateTime(year(), month(), day(), hour(), minute(), second()) );
      Serial.print("    -> Relecture de l'heure RTC : ");
      dateheure_rtc = rtc.now();
      affiche_heure(dateheure_rtc, AFFICHE_JOUR_SEMAINE, AFFICHE_DATE, RETOURLIGNE);
    }  
    else {
      Serial.print("   -> OK (moins de ");Serial.print(ECART_MAX); Serial.println("s d'écart.)");
      allumage_led_progression( etape * NB_LED_PAR_ETAPE, NB_LED_PAR_ETAPE , CRGB::Green);
   } 
  } else {
    Serial.println("Synchro impossible !");
    allumage_led_progression( etape * NB_LED_PAR_ETAPE, NB_LED_PAR_ETAPE , CRGB::Red);
  } 
 
  etape++;
  
  // initialisation et vérification de présence du capteur de luminosité
  allumage_led_progression( etape * NB_LED_PAR_ETAPE, NB_LED_PAR_ETAPE , CRGB::Yellow);
  Serial.print(etape+1);
  Serial.print("-> Détection du capteur de luminosité BH1750 : ");
  if (etape %2) {digitalWrite(LED_BUILTIN, HIGH);} else {digitalWrite(LED_BUILTIN, LOW);}
  if (BH1750sensor.begin(BH1750::CONTINUOUS_LOW_RES_MODE)) { // choix du mode de fonctionnement du capteur (vitesse, précision)
    Serial.println(F(" -> BH1750 configuré !"));
    bh=true;
    BH1750sensor.readLightLevel();  //cette première lecture permet d'éviter d'avoir une mesure à 0 la première fois (bug ?)
    allumage_led_progression( etape * NB_LED_PAR_ETAPE, NB_LED_PAR_ETAPE , CRGB::Green);
  }
  else {
    Serial.println(F("  -> Erreur d'initialisation BH1750 !"));
    bh=false;
    allumage_led_progression( etape * NB_LED_PAR_ETAPE, NB_LED_PAR_ETAPE , CRGB::Red);
  }

  etape++;
  Serial.println("Fin de la configuration ...");
  Serial.println();
  int led_restantes = NUM_LEDS -  etape * NB_LED_PAR_ETAPE;
  if (led_restantes >0 ) {allumage_led_progression( etape * NB_LED_PAR_ETAPE, led_restantes, CRGB::Green);}  //allumage des 4 dernières LED}
  
  delay(1000);
  digitalWrite(LED_BUILTIN, HIGH);
  FastLED.show();
}

// *******************************************************
// *                  BOUCLE PRINCIPALE                  *
// *******************************************************
void loop() {
  // permet la MàJ via OTA
  #ifdef MAJ_OTA 
  ArduinoOTA.handle();
  #endif
  // on lit les capteurs au boot
  if (boot) {
    if (bh) {lecture_BH1750(NO_RETOURLIGNE);}
    if (dht) {lecture_DHT(NO_RETOURLIGNE);}
    for (int i=0; i<=30;i++) {          //on initialise le tableau avec la première mesure
      histo_temp[i] = temperature;  
    }
    boot=false;
  }
  
  //affichage de l'heure si nécessaire
  DateTime date;
  if ((!ntp) && (dsrtc)) {  // on doit faire confiance à l'horloge RTC vu que NTP ne fonctionne pas
    date=rtc.now();
  } else {
     date=now();        // // on fait confiance à l'heure NTP
  }
  Hheure=date.hour();
  Mminute=date.minute();
  Sseconde=date.second();
  #ifndef CLIGNOTEMENT_SEPARATEURS 
    affichage_ruban_separateur(SEPARATEUR_HM, couleurSEPHM);
    affichage_ruban_separateur(SEPARATEUR_MS, couleurSEPMS);
  #endif
  #ifdef CHANGEMENT_HEURE_SAISONNIER
  if (  (Hheure == Hsaison || Hheure == Hsaison +1) && (Mminute==Msaison) && ((Sseconde == Ssaison) || (Sseconde == Ssaison +10)  ) ) {  // mise à l'heure en cas de changement d'heure saisonnier) (tentative à +10s si jamais il y a eu l'affichage de la température/humidité au mauvais moment)
    Serial.println("");
    Serial.println("Mise à l'heure saisonnière.");
    NTP.getTime();
    delay(1100);    // pour ne plus être dans la même seconde et ne plus refaire la mise à jour de l'heure (et laisse le temps au serveur de répondre)
  }
  #endif

  
  if (old_Sseconde != Sseconde) {
    old_Sseconde=Sseconde;
    timerTH++;
    affichage_ruban_secondes(Sseconde, couleurDS,couleurUS);
    affichage_ruban_heures(Hheure, couleurDH,couleurUH);
    affichage_ruban_minutes(Mminute, couleurDM,couleurUM);
    #ifdef CLIGNOTEMENT_SEPARATEURS //clignotement des séparateurs
      if (Sseconde%2) {
        affichage_ruban_separateur(SEPARATEUR_HM, couleurSEPHM);
        affichage_ruban_separateur(SEPARATEUR_MS, couleurSEPMS);
      } else {
        affichage_ruban_separateur(SEPARATEUR_HM, CRGB::Black);
        affichage_ruban_separateur(SEPARATEUR_MS, CRGB::Black);
      }   
    #endif
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    // Affichage des données sur le port série
    affiche_heure(date, NO_AFFICHE_JOUR_SEMAINE, NO_AFFICHE_DATE,NO_RETOURLIGNE);
    Serial.print(" ");
    
    // on lit la luminosité et on adapte le niveau d'éclairage des LED (brightness)
    if (bh) {
      lecture_BH1750(NO_RETOURLIGNE);
        
      led_luminosite = remapping(luminosite*COEFF_LUM,LUM_AMBIANTE_MIN,LUM_AMBIANTE_MAX,LUM_MINI,LUM_MAXI);   //transforme la luminosité en une valeur brightness pour le ruban de LED
      if (luminosite==-1) {led_luminosite=LUM_MAXI/2;}
      if (led_luminosite < LUM_MINI) {led_luminosite=LUM_MINI;}
      if (led_luminosite > LUM_MAXI) {led_luminosite=LUM_MAXI;}
      Serial.print(" Brightness = ");Serial.print(led_luminosite);Serial.print(" ");
    } else {
      couleurDH=CRGB::DarkOrange;
      couleurUH=CRGB::DarkOrange;
      couleurDM=CRGB::DarkOrange;
      couleurUM=CRGB::DarkOrange;
      couleurDS=CRGB::DarkOrange;
      couleurUS=CRGB::DarkOrange;
      couleurSEPHM=CRGB::DarkOrange;
      couleurSEPMS=CRGB::DarkOrange;
      FastLED.setBrightness(LUM_MAXI /2);
    }

    // on lit la température et on adapte la couleur des LED (Hue dans l'espace HSV)
    if (dht) {
        lecture_DHT(NO_RETOURLIGNE);
        led_temperature = calcul_couleur_temp(temperature);
        Serial.print(" Hue = ");Serial.print(led_temperature);

        // // on calcule les couleurs pour les 6 digits 
        FastLED.setBrightness(LUM_MAXI);
        CHSV hsvUS( led_temperature, 255 , led_luminosite);                      
        hsv2rgb_rainbow( hsvUS, couleurUS);
        CHSV hsvDS(calcul_couleur_temp(histo_temp[5]), 255 , led_luminosite);
        hsv2rgb_rainbow(hsvDS, couleurDS);
        CHSV hsvUM(calcul_couleur_temp(histo_temp[11]), 255 , led_luminosite);
        hsv2rgb_rainbow(hsvUM, couleurUM);
        CHSV hsvDM(calcul_couleur_temp(histo_temp[17]), 255 , led_luminosite);
        hsv2rgb_rainbow(hsvDM, couleurDM);
        CHSV hsvUH(calcul_couleur_temp(histo_temp[23]), 255 , led_luminosite);
        hsv2rgb_rainbow(hsvUH, couleurUH);
        CHSV hsvDH(calcul_couleur_temp(histo_temp[29]), 255 , led_luminosite);
        hsv2rgb_rainbow(hsvDH, couleurDH);
        CHSV hsvSEPMS(calcul_couleur_temp(histo_temp[8]), 255 , led_luminosite);
        hsv2rgb_rainbow(hsvSEPMS, couleurSEPMS);
        CHSV hsvSEPHM(calcul_couleur_temp(histo_temp[20]), 255 , led_luminosite);
        hsv2rgb_rainbow(hsvSEPHM, couleurSEPHM);
       
    } else {            // en cas de valeur anormale
      couleurDH=CRGB::Red;
      couleurUH=CRGB::Red;
      couleurDM=CRGB::Red;
      couleurUM=CRGB::Red;
      couleurDS=CRGB::Red;
      couleurUS=CRGB::Red;
      couleurSEPHM=CRGB::Red;
      couleurSEPMS=CRGB::Red;
      FastLED.setBrightness(LUM_MAXI /2);
      lecture_DHT(NO_RETOURLIGNE);                // relance une lecture, si jamais ça remarche
    }
      
      Serial.print("                  ");

    if  (timerTH >= DELAI_AFFICHAGE_TEMP_HUM ) {   // on affiche humidité et température toutes les DELAI_AFFICHAGE_TEMP_HUM s (bien suffisant)

      timerTH=0;
      #ifdef AFFICHE_TEMPERATURE
        affichage_ruban_temperature();
        FastLED.show();
        delay(TEMPS_AFFICHAGE);
        #endif
        #ifdef AFFICHE_HUMIDITE
        if (affiche_humidite) {
          affichage_ruban_humidite();
          FastLED.show();
          delay(TEMPS_AFFICHAGE);
        }
        affiche_humidite=!affiche_humidite;
        #endif
      
      
      
      
    }
    for (int i=0; i<160; i++) {Serial.write(BACKSPACE);}
    FastLED.show();
  }


  if (old_Mminute != Mminute) {
    old_Mminute = Mminute;
    if ((Mminute % HISTO_TEMP_DELAI == 0) && (dht)) {         // toutes les HISTO_TEMP_DELAI minutes on change la couleur des LED
      // on décale les données dans le tableau
      for (int i=0; i<=29;i++) {
          histo_temp[30-i]=histo_temp[30-i-1];    // on décale 
      }
      histo_temp[0]=temperature;            // on mémorise la dernière mesure
     // affichage_histo();
    }

    #ifdef BIPS
    if (Mminute == 0) {                                       /// à chaque heure pleine
      bip(DUREE_BIP_00);
    }
    if (Mminute == 30) {
      bip(DUREE_BIP_30);
    }
    #endif
  }
  // gestion des requetes HTTP
  server.handleClient();


 }