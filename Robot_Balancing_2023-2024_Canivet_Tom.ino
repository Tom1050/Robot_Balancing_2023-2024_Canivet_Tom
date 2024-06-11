#include <MPU6050.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <ESP32Servo.h>
#include <BluetoothSerial.h>

#define _10ms 10

float ACCEL_YANGLE;         //variable de l'axe X et Y de l'accéléromètre du MPU6050
float x_out, y_out, z_out;  //variable de la sortie de l'axe X et Y de l'accéléromètre du MPU6050

MPU6050 mpu;     //initialisation du MPU6050
Servo myservo1;  //initilisation du moteur 1
Servo myservo2;  //initilisation du moteur 2

double Setpoint, Input;

int vd_1s = 0;
int vd_20ms = 0;
int vd_250ms = 0;
int vd_260ms = 0;
int vd_100ms = 0;

int aerreur = 0;
int berreur = 0;
int erreur1 = 0;
int erreur2 = 0;
int consigne = 0;


unsigned long previousMillis = 0;  //variable de la précédente millis
// Complementary filter variables
float angle = 0;     // Filtered angle / variable de la valeur de l'angle
float gyroRate = 0;  // Gyro rate / initalisation de la vitesse du gyroscope
float alpha = 0.98;  // Filter coefficient / variable du coéficient du filtre

// Bluetooth variables
const char *pin = "1234";             // code pour le bluetooth du micrcontrôleur
String myName = "robot_balance";      //
String slaveName = "ESP32-BT-Slave";  // le nom qui apparaît dans le bluetooth
BluetoothSerial SerialBT;             //initialisation du bluetooth du microcontrôleur

void Get_Accel_Angles() {

  int16_t ax, ay, az, gx, gy, gz;                // variable de l'accéléromètre et du gyscope avec l'axe X, Y et Z
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);  //erreur ici

  //Accelerometer data
  x_out = ax / 16384.0;
  y_out = ay / 16384.0;
  z_out = az / 16384.0;

  ACCEL_YANGLE = 57.295 * atan(-x_out / sqrt(pow(z_out, 2) + pow(y_out, 2)));

  //Gyroscope data
  gyroRate = gy / 65.5;  // Convert gyro rate to degrees per second

  // Complementary filter
  unsigned long currentMillis = millis();
  float dt = (currentMillis - previousMillis) / 1000.0;
  previousMillis = currentMillis;

  angle = alpha * (angle + gyroRate * dt) + (1 - alpha) * ACCEL_YANGLE;
}


int limite(int val, int lim_haute, int lim_basse) {
  if (val > lim_haute)
    return lim_haute;
  else if (val < lim_basse)
    return lim_basse;
  else
    return val;
}


#define LED 13
unsigned long tempsMemoriser;  // Pour mémoiriser le temps
bool Etat_LED = 0;

void setup() {
  pinMode(LED, OUTPUT);
  tempsMemoriser = millis();  // Initialisation du temps t
  Wire.begin();
  Serial.begin(115200);
  SerialBT.begin(slaveName);
  myservo1.attach(21);
  myservo2.attach(19);
  delay(1);

  mpu.initialize();
  if (mpu.testConnection()) {
    Serial.println("MPU6050 connection successful");
  } else {
    Serial.println("MPU6050 connection failed");
    while (1)
      ;
  }
}

void loop() {

  if (millis() >= tempsMemoriser + _10ms)  // si millis est égale au temps Mémoriser plus 10millisecond
  {

    tempsMemoriser = millis();  // Mise à jour de t pour le prochain passage


    Get_Accel_Angles();

    if (vd_1s++ >= 100)  //boucle de 1 seconde
    {
      vd_1s = 0;
      Etat_LED = !Etat_LED;  // Inversion de la variable Etat_LED (0 ==> 1 ==> 0 ==>...)
      digitalWrite(LED, Etat_LED);
    }

    /*  if (vd_10ms++ >= 1)  //boucle toute les 40millisecond
    {
      vd_10ms = 0;
      tempChute = (erreur1*);
    }*/

    if (vd_20ms++ >= 1)  //boucle toute les 20millisecond pour commander les servomoteurs
    {

      int tempCalcul = 0;

      vd_20ms = 0;
      Get_Accel_Angles();
      erreur1 = int(87 - ACCEL_YANGLE);
      erreur2 = int(91 + ACCEL_YANGLE);

      tempCalcul = int(((erreur1 - 90) * 10) - aerreur);

      erreur1 = int(limite(erreur1 + tempCalcul, 180, 0));
      erreur2 = int(limite(erreur2 - tempCalcul, 180, 0));

      myservo2.write(erreur2);
      myservo1.write(erreur1);
      berreur = ACCEL_YANGLE;
    }

    if (vd_250ms++ >= 25)  //boucle toute les 500millisecond
    {
      vd_250ms = 0;
      aerreur = berreur;
      aerreur = aerreur - erreur1;
    }
     if (vd_260ms++ >= 26)  //boucle toute les 500millisecond
    {
      vd_260ms = 0;
      //aerreur = 0;
    }
    if (vd_100ms++ >= 10)  //boucle toute les 100millisecond
    {
      vd_100ms = 0;

      Serial.printf("\n YANGLE : %07.3f err1:%d err2:%d, aerreur:%d", ACCEL_YANGLE, erreur1, erreur2, aerreur);
      SerialBT.printf("\n YANGLE : %07.3f err1:%d err2:%d", ACCEL_YANGLE, erreur1, erreur2);
    }
  }
}