#include <MPU6050.h>  
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>  
#include <ESP32Servo.h>
#include <PID_v1.h>
#include <BluetoothSerial.h>  

float ACCEL_XANGLE, ACCEL_YANGLE;  //variable de l'axe X et Y de l'accéléromètre du MPU6050
float x_out, y_out, z_out; //variable de la sortie de l'axe X et Y de l'accéléromètre du MPU6050
float ACCEL_YANGLE_mod;
int consigne_moteur = 0, consigne_moteur_g = 0, consigne_moteur_d = 0; //variable de la consigne du moteur droite et du moteur gauche
Servo myservo1; //initilisation du moteur 1
Servo myservo2; //initilisation du moteur 2
MPU6050 mpu; //initialisation du MPU6050

float ecart;
float derive;
int consigne = 0;

// PID variables
double Setpoint, Input, Output; //
double Kp = 2, Ki = 5, Kd = 1; // PID coefficients (adjust these values as needed)
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT); //intialisation des PID

// Bluetooth variables
const char *pin = "1234";  // code pour le bluetooth du micrcontrôleur
String myName = "robot_balance"; // 
String slaveName = "ESP32-BT-Slave"; // le nom qui apparaît dans le bluetooth  
BluetoothSerial SerialBT;  //initialisation du bluetooth du microcontrôleur

// Timing variables
unsigned long previousMillis = 0; //variable de la précédente millis
const long interval = 10; // intervalle de boucle en milliseconde

// Complementary filter variables
float angle = 0; // Filtered angle / variable de la valeur de l'angle
float gyroRate = 0; // Gyro rate / initalisation de la vitesse du gyroscope
float alpha = 0.98; // Filter coefficient / variable du coéficient du filtre

void Get_Accel_Angles() {
  int16_t ax, ay, az, gx, gy, gz; // variable de l'accéléromètre et du gyscope avec l'axe X, Y et Z
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // Accelerometer data
  x_out = ax / 16384.0;  
  y_out = ay / 16384.0;  
  z_out = az / 16384.0;  

  ACCEL_XANGLE = 57.295 * atan(y_out / sqrt(pow(z_out, 2) + pow(x_out, 2)));
  ACCEL_YANGLE = 57.295 * atan(-x_out / sqrt(pow(z_out, 2) + pow(y_out, 2)));

  // Gyroscope data
  gyroRate = gy / 65.5; // Convert gyro rate to degrees per second

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
//mdr enfaite non
void setup() {
  Wire.begin();          
  Serial.begin(115200);  
//  SerialBT.begin(slaveName);  
  myservo1.attach(19);
  myservo2.attach(21);
  delay(1);

  mpu.initialize();
  if (mpu.testConnection()) 
  {
    Serial.println("MPU6050 connection successful");
    SerialBT.println("MPU6050 connection successful");  
  } else {
    Serial.println("MPU6050 connection failed");
    SerialBT.println("MPU6050 connection failed");  
    while (1);  
  }

  Setpoint = 0; // Desired angle
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(-255, 255); // Adjust to the range of your motors
}

void loop() {
  Get_Accel_Angles();
  
  Input = angle;

  myPID.Compute();

  consigne_moteur_g = limite(int(92 + Output), 140, -140);
  consigne_moteur_d = limite(int(100 - Output), 140, -140);

  myservo2.write(consigne_moteur_g);
  myservo1.write(consigne_moteur_d);

  Serial.printf("\n#servo2:%d", consigne_moteur_g);
  SerialBT.printf("\n#servo2:%d", consigne_moteur_g);  
}
