#include <MPU6050.h>  // Inclure la bibliothèque MPU6050 adaptée à Arduino ou ESP32
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>  //librairie I2C
#include <ESP32Servo.h>
#define MPU_ADDRESS 0x68 
float ACCEL_XANGLE, ACCEL_YANGLE, ACCEL_ZANGLE;  // Déclaration des variables globales
float x_out, y_out, z_out;
Servo myservo1;
Servo myservo2;
MPU6050 mpu;
int pos = 0;
int FS_SEL = 1;
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

  ACCEL_XANGLE = 57.295 * atan(y_out / sqrt(pow(z_out, 2) + pow(x_out, 2)));            //Je ne comprend pas cette ligne de code
  ACCEL_YANGLE = 57.295 * atan(-x_out / sqrt(pow(z_out, 2) + pow(y_out, 2))) * FS_SEL;  //Je ne comprend pas cette ligne de code
  ACCEL_ZANGLE = 57.295 * atan(sqrt(pow(x_out, 2) + pow(y_out, 2)) / z_out);            //Je ne comprend pas cette ligne de code
}
void setupMpu6050Registers() {
    // Configure power management
    Wire.beginTransmission(MPU_ADDRESS); // Start communication with MPU
    Wire.write(0x6B);                    // Request the PWR_MGMT_1 register
    Wire.write(0x00);                    // Apply the desired configuration to the register
    Wire.endTransmission();              // End the transmission

    // Configure the gyro's sensitivity
    Wire.beginTransmission(MPU_ADDRESS); // Start communication with MPU
    Wire.write(0x1B);                    // Request the GYRO_CONFIG register
    Wire.write(0x20);                    // Apply the desired configuration to the register : ±500°/s
    Wire.endTransmission();              // End the transmission

    // Configure the acceleromter's sensitivity
    Wire.beginTransmission(MPU_ADDRESS); // Start communication with MPU
    Wire.write(0x1C);                    // Request the ACCEL_CONFIG register
    Wire.write(0x40);                    // Apply the desired configuration to the register : ±8g
    Wire.endTransmission();              // End the transmission

    // Configure low pass filter
    Wire.beginTransmission(MPU_ADDRESS); // Start communication with MPU
    Wire.write(0x1A);                    // Request the CONFIG register
    Wire.write(0x03);                    // Set Digital Low Pass Filter about ~43Hz
    Wire.endTransmission();              // End the transmission
}
void setup() {
  Wire.begin();          // Initialiser la communication I2C
  Serial.begin(115200);  // Initialiser la communication série
  myservo1.attach(21);
  myservo2.attach(19);
  delay(1);
  // Initialisation du MPU6050
  mpu.initialize();
  if (mpu.testConnection()) {
    Serial.println("MPU6050 connection successful");
  } else {
    Serial.println("MPU6050 connection failed");
    while (1)
      ;  // Arrêter le programme si la connexion a échoué
  }
  setupMpu6050Registers();
}
// Autres initialisations...
//coucou daoud
void loop() {
  // Votre code ici...
  // Appel de la fonction Get_Accel_Angles
  Get_Accel_Angles();
  Serial.printf("agx:%07.3f acx:%07.3f agy:%07.3f acy:%07.3f agz:%07.3f acz:%07.3f\n", ACCEL_XANGLE, x_out, ACCEL_YANGLE, y_out, ACCEL_ZANGLE, z_out);
  ecart = float(consigne - ACCEL_YANGLE);
  Serial.println();
  Serial.println(ecart);  // Ajouter des délais appropriés pour vos besoins

  myservo2.write(int(92 + ecart))*(int 2);
  Serial.printf("ecart :%07.3f", ecart);
  Serial.println();

  myservo1.write(int(95 - ecart));
  Serial.printf("ecart :%07.3f", ecart);
  Serial.println();
}