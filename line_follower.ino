#include <AFMotor.h>
#include <QTRSensors.h>
#include <Servo.h>
 
AF_DCMotor motor3(3, MOTOR34_8KHZ );
AF_DCMotor motor4(4, MOTOR34_8KHZ );
 
#define KP .2
#define KD 5
#define M3_minumum_hiz 120
#define M4_minumum_hiz 120
#define M3_maksimum_hiz 255
#define M4_maksimum_hiz 255
#define MIDDLE_SENSOR 4
#define NUM_SENSORS 8
#define TIMEOUT 2500
#define EMITTER_PIN 53
#define DEBUG 0
Servo mano;
Servo seguro;
const int t = 35;
const int e = 33;

 
QTRSensorsRC qtrrc((unsigned char[]) {51, 49, 47, 45, 43, 41, 39, 37} ,NUM_SENSORS, TIMEOUT, EMITTER_PIN);
 
unsigned int sensorValues[NUM_SENSORS];
 
void setup() {
  mano.attach(10);
  seguro.attach(9);
  delay(1500);
  manual_calibration();
  set_motors(0,0);
  pinMode(t,OUTPUT);
  pinMode(e,INPUT);
  Serial.begin(9600);
}

int lastError = 0;
int last_proportional = 0;
int integral = 0;
int count=0;
int counte=0;

void loop(){
  long duracion, distancia;
  digitalWrite(t, LOW);
  delayMicroseconds(2);
  digitalWrite(t, HIGH);
  delayMicroseconds(10);
  digitalWrite(t, LOW);
  duracion = pulseIn(e,HIGH);
  distancia = duracion/29.1/2;
  Serial.println(distancia);
  mano.write(0); 
  if (distancia<=11 && count==0) {  //cuando llega al led (+100)
    motor3.setSpeed(200);
    motor4.setSpeed(200);
    motor3.run(FORWARD);
    motor4.run(FORWARD);
    delay(200);
    motor3.run(BACKWARD);
    motor4.run(BACKWARD);
    delay(1000);
    motor3.run(FORWARD);
    motor4.run(BACKWARD);
    delay(120);
    motor3.run(FORWARD);
    motor4.run(FORWARD);
    count=1;
    mano.write(50);
  } else if (distancia<=11 && count==1){ // depositar las bolas
      motor3.setSpeed(0);
      motor4.setSpeed(0);
      motor3.run(BACKWARD);
      motor4.run(BACKWARD);
      seguro.write(170);
      delay(13000);
      motor3.setSpeed(100);
      motor3.setSpeed(100);
      motor3.run(BACKWARD);
      delay(1000);
    }
  if(digitalRead(51)>=100 && digitalRead(37)>=100 && counte==0) { //giro a la izquierda
    motor4.setSpeed(0);
    delay(500);
    counte=1;
  } else if (digitalRead(51)>=100 && digitalRead(37)>=100 && counte==1) { //continuar derecho
    motor3.setSpeed(120);
    motor4.setSpeed(120);
    delay(700);
    mano.write(50);
    counte=2;
  }else if (digitalRead(51)>=100 && digitalRead(37)>=100 && counte==2) { //giro a las derecha
    motor3.setSpeed(0);
    delay(500);
    mano.write(50);
  }
  unsigned int sensors[NUM_SENSORS];
  int position = qtrrc.readLine(sensors);
  int error = position - 2000;
  int motorSpeed = KP * error + KD * (error - lastError);  
  lastError = error;
  int leftMotorSpeed = M3_minumum_hiz + motorSpeed;
  int rightMotorSpeed = M4_minumum_hiz - motorSpeed;
  // set motor speeds using the two motor speed variables above
  set_motors(leftMotorSpeed, rightMotorSpeed);
}

void set_motors(int motor3speed, int motor4speed) {
  if (motor3speed > M3_maksimum_hiz ) motor3speed = M3_maksimum_hiz; //MAKSİMUM MOTOR 1 HIZ LİMİTİ
  if (motor4speed > M4_maksimum_hiz ) motor4speed = M4_maksimum_hiz; // MAKSİMUM MOTOR 2 HIZ LİMİTİ
  if (motor3speed < 0) motor3speed = 0; // MİNIMUMMOTOER 1 HIZ LİMİTİ
  if (motor4speed < 0) motor4speed = 0; // MİNİMUM MOTOR 2 HIZ LİMİTİ
  motor3.setSpeed(motor3speed); //1.MOTOR HIZI
  motor4.setSpeed(motor4speed);// 2.MOTOR HIZI
  motor3.run(FORWARD); //İLERİ
  motor4.run(FORWARD); //İLERİ
}

void manual_calibration() {
  Serial.begin(9600);
  int i;
  for (i = 0; i < 250; i++){
    qtrrc.calibrate(QTR_EMITTERS_ON);
    delay(20);
 }
 if (DEBUG) {
   Serial.begin(9600);
   for (int i = 0; i < NUM_SENSORS; i++) { 
     Serial.print(qtrrc.calibratedMinimumOn[i]);
     Serial.print(' ');
   }
   Serial.println();
   for (int i = 0; i < NUM_SENSORS; i++) {
     Serial.print(qtrrc.calibratedMaximumOn[i]);
     Serial.print(' ');
   }
   Serial.println();
   Serial.println();
 }
 }
