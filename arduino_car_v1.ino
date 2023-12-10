
//-------Ultrasonido--------------
#define TRIG_PIN 13  
#define ECHO_PIN 12  
//--------------------------------

//-------Inflarrojo---------------
#define PIN_ITR20001-LEFT   A2
#define PIN_ITR20001-MIDDLE A1
#define PIN_ITR20001-RIGHT  A0

// Umbral de detección de línea
//Cuando detecta por ejemplo blanco el valor está entre 800 y 900 
//pero cuando detecta negro el valor baja mucho. Ponemos este umbral:
const int threshold = 700;

int left_val;
int right_val;
int mid_val;
//--------------------------------

//-------Motor------------------
// Enable/Disable motor control.
//  HIGH: motor control enabled
//  LOW: motor control disabled
#define PIN_Motor_STBY 3

// Group A Motors (Right Side)
// PIN_Motor_AIN_1: Digital output. HIGH: Forward, LOW: Backward
#define PIN_Motor_AIN_1 7
// PIN_Motor_PWMA: Analog output [0-255]. It provides speed.
#define PIN_Motor_PWMA 5

// Group B Motors (Left Side)
// PIN_Motor_BIN_1: Digital output. HIGH: Forward, LOW: Backward
#define PIN_Motor_BIN_1 8
// PIN_Motor_PWMB: Analog output [0-255]. It provides speed.
#define PIN_Motor_PWMB 6
//-----------------------------

// Function to control motors
void motorControl(bool motorAForward, int speedA, bool motorBForward, int speedB) {
  // Control motor A (RIGHT)
  digitalWrite(PIN_Motor_AIN_1, motorAForward ? HIGH : LOW);
  analogWrite(PIN_Motor_PWMA, speedA);

  // Control motor B (LEFT)
  digitalWrite(PIN_Motor_BIN_1, motorBForward ? HIGH : LOW);
  analogWrite(PIN_Motor_PWMB, speedB);
}


void sensorReading(){
  left_val = analogRead(A2); // read from left Sensor
  right_val = analogRead(A0);
  mid_val =  analogRead(A1)
}

void turnLeft(){
  motorControl(true,175,false,0);
}
void turnRight(){
  motorControl(false,0,true,175);
}
void forward(){
  motorControl(true,175,true,175);
}
void stop_motors(){
  motorControl(false,0,false,0);
}
//void recovery(){
//  motorControl(false,0,false,200);
//}

int ping(int TriggerPin, int EchoPin) {
  long duration, distanceCm;
  
  digitalWrite(TriggerPin, LOW);  //para generar un pulso limpio ponemos a LOW 4us
  delayMicroseconds(4);
  digitalWrite(TriggerPin, HIGH);  //generamos Trigger (disparo) de 10us
  delayMicroseconds(10);
  digitalWrite(TriggerPin, LOW);
  
  duration = pulseIn(EchoPin, HIGH);  //medimos el tiempo entre pulsos, en microsegundos
  
  distanceCm = duration * 10 / 292/ 2;   //convertimos a distancia, en cm
  return distanceCm;
}


void setup() {
  //ultrasonidos
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  
  //inflarrojos
  pinMode(PIN_ITR20001-LEFT, INPUT);
  pinMode(PIN_ITR20001-MIDDLE, INPUT);
  pinMode(PIN_ITR20001-RIGHT, INPUT);
  //motors
  // Turn on the engines !!!!
  digitalWrite(PIN_Motor_STBY, HIGH);
  // Set motor control pins as outputs
  pinMode(PIN_Motor_STBY, OUTPUT);
  pinMode(PIN_Motor_AIN_1, OUTPUT);
  pinMode(PIN_Motor_PWMA, OUTPUT);
  pinMode(PIN_Motor_BIN_1, OUTPUT);
  pinMode(PIN_Motor_PWMB, OUTPUT);

  Serial.begin(9600); // // Serial Communication is starting with 9600 of baudrate speed

}



void loop() {
  sensorReading();
  int distance = ping(TRIG_PIN,ECHO_PIN);
  
  if(leftValue <= threshold && rightValue >= threshold){
    turnLeft();
  }else if(leftValue >= threshold && rightValue <= threshold ){
    turnRight();
  }else if(leftValue <= threshold && rightValue <= threshold){
    forward();
  }else if(distance < 20){
    stop_motors()
  }//else if(leftValue == 1 && rightValue == 1){
    //recovery();
  //}
}
