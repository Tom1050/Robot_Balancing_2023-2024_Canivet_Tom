#include <MPU6050.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <ESP32Servo.h>

#define _10ms 10

float ACCEL_YANGLE1;  //variable de l'axe X et Y de l'accéléromètre du MPU6050
float ACCEL_YANGLE2;
float x_out, y_out, z_out;  //variable de la sortie de l'axe X et Y de l'accéléromètre du MPU6050

MPU6050 mpu;     //initialisation du MPU6050
Servo myservo1;  //initilisation du moteur 1
Servo myservo2;  //initilisation du moteur 2

double Setpoint, Input;

int vd_1s = 0;
int vd_20ms = 0;
int vd_40ms = 0;
int vd_100ms = 0;

int erreur1 = 0;
int erreur2 = 0;
int consigne = 0;

int integral = 0;

unsigned long previousMillis = 0;  //variable de la précédente millis
// Complementary filter variables
float angle1 = 0;
float angle2 = 0;    // Filtered angle / variable de la valeur de l'angle
float gyroRate = 0;  // Gyro rate / initalisation de la vitesse du gyroscope
float alpha = 0.98;  // Filter coefficient / variable du coéficient du filtre


void Get_Accel_Angles1() {

  int16_t ax, ay, az, gx, gy, gz;                // variable de l'accéléromètre et du gyscope avec l'axe X, Y et Z
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);  //erreur ici

  //Accelerometer data
  x_out = ax / 16384.0;
  y_out = ay / 16384.0;
  z_out = az / 16384.0;

  ACCEL_YANGLE1 = 57.295 * atan(-x_out / sqrt(pow(z_out, 2) + pow(y_out, 2)));

  //Gyroscope data
  gyroRate = gy / 65.5;  // Convert gyro rate to degrees per second

  // Complementary filter
  unsigned long currentMillis = millis();
  float dt = (currentMillis - previousMillis) / 1000.0;
  previousMillis = currentMillis;

  angle1 = alpha * (angle1 + gyroRate * dt) + (1 - alpha) * ACCEL_YANGLE1;
}


void Get_Accel_Angles2() {

  int16_t ax, ay, az, gx, gy, gz;                // variable de l'accéléromètre et du gyscope avec l'axe X, Y et Z
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);  //erreur ici

  //Accelerometer data
  x_out = ax / 16384.0;
  y_out = ay / 16384.0;
  z_out = az / 16384.0;

  ACCEL_YANGLE2 = 57.295 * atan(-x_out / sqrt(pow(z_out, 2) + pow(y_out, 2)));

  //Gyroscope data
  gyroRate = gy / 65.5;  // Convert gyro rate to degrees per second

  // Complementary filter
  unsigned long currentMillis = millis();
  float dt = (currentMillis - previousMillis) / 1000.0;
  previousMillis = currentMillis;

  angle2 = alpha * (angle2 + gyroRate * dt) + (1 - alpha) * ACCEL_YANGLE2;
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


    Get_Accel_Angles1();

    if (vd_1s++ >= 100)  //boucle de 1 seconde
    {
      vd_1s = 0;
      Etat_LED = !Etat_LED;  // Inversion de la variable Etat_LED (0 ==> 1 ==> 0 ==>...)
      digitalWrite(LED, Etat_LED);
    }

    /* if (vd_10ms++ >= 1)  //boucle toute les 40millisecond
    {
      vd_10ms = 0;
      tempChute = (erreur1 * );
    }*/

    if (vd_20ms++ >= 2)  //boucle toute les 20millisecond pour commander les servomoteurs
    {

      int tempCalcul1 = 0;
      int tempCalcul2 = 0;
      vd_20ms = 0;
      Get_Accel_Angles2();
      erreur1 = int(87.2 - ACCEL_YANGLE1);
      erreur2 = int(98.3 + ACCEL_YANGLE2);

      integral = int(ACCEL_YANGLE2 * 2.5 - ACCEL_YANGLE1);

      tempCalcul1 = int(erreur1 + 90) * integral;
      tempCalcul2 = int(erreur1 +  90) * integral;

      erreur1 = int(limite(erreur1 - tempCalcul2, 180, 0));
      erreur2 = int(limite(erreur2 + tempCalcul1, 180, 0));




      myservo2.write(erreur2);
      myservo1.write(erreur1);
    }

    if (vd_40ms++ >= 4)  //boucle toute les 40millisecond
    {
      vd_40ms = 0;
    }
    if (vd_100ms++ >= 10)  //boucle toute les 100millisecond
    {
      vd_100ms = 0;

      Serial.printf("\n YANGLE1 : %07.3f YANGLE2 : %07.3f err1:%d err2:%d integral : %07.3f", ACCEL_YANGLE1, ACCEL_YANGLE2, erreur1, erreur2, integral);
    }
  }
}