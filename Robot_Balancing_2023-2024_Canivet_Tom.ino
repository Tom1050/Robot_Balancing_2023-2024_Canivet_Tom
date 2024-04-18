#include <MPU6050.h>  // Inclure la bibliothèque MPU6050 adaptée à Arduino ou ESP32
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>  //librairie I2C
#include <ESP32Servo.h>

float ACCEL_XANGLE, ACCEL_YANGLE, ACCEL_ZANGLE;  // Déclaration des variables globales
float x_out, y_out, z_out;
Servo myservo1;
Servo myservo2;
MPU6050 mpu;
int pos = 0;
//int pos- < 0
float ecart;
int consigne = 0;
void Get_Accel_Angles() {
  // Lire les données de l'accéléromètre
  int16_t ax, ay, az;
  mpu.getAcceleration(&ax, &ay, &az);

  // Calculer les angles
  x_out = ax / 16384.0;  // Conversion de l'accélération brute en g
  y_out = ay / 16384.0;  // Conversion de l'accélération brute en g
  z_out = az / 16384.0;  // Conversion de l'accélération brute en g

  ACCEL_XANGLE = 57.295 * atan(y_out / sqrt(pow(z_out, 2) + pow(x_out, 2)));
  ACCEL_YANGLE = 57.295 * atan(-x_out / sqrt(pow(z_out, 2) + pow(y_out, 2)));
  ACCEL_ZANGLE = 57.295 * atan(sqrt(pow(x_out, 2) + pow(y_out, 2)) / z_out);
}

void setup() {
  Wire.begin();          // Initialiser la communication I2C
  Serial.begin(115200);  // Initialiser la communication série
  myservo1.attach(21);
  myservo2.attach(19);
  delay(100);
  // Initialisation du MPU6050
  mpu.initialize();
  if (mpu.testConnection()) {
    Serial.println("MPU6050 connection successful");
  } else {
    Serial.println("MPU6050 connection failed");
    while (1)
      ;  // Arrêter le programme si la connexion a échoué
  }

  // Autres initialisations...
}
//coucou daoud
void loop() {
  // Votre code ici...
  // Appel de la fonction Get_Accel_Angles
  Get_Accel_Angles();
  Serial.printf("agx:%07.3f acx:%07.3f agy:%07.3f acy:%07.3f agz:%07.3f acz:%07.3f\n", ACCEL_XANGLE, x_out, ACCEL_YANGLE, y_out, ACCEL_ZANGLE, z_out);
  ecart = float(consigne - ACCEL_YANGLE);
  Serial.println();
  Serial.println(ecart);  // Ajouter des délais appropriés pour vos besoins

  myservo2.write(int(97 + ecart)); 
  Serial.printf("ecart :%07f", ecart);
  Serial.println();

  myservo1.write(int(90 - ecart)); 
  Serial.printf("ecart :%07f", ecart);
  Serial.println();
}