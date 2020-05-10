//  DC Motor Control with PWM for H-Bridge board L298N 
//  By Ross Milligan 
//  21/02/2020

//connections for left wheels motor
#define IN1 PA12    //2 //(D2)
#define IN2 PB7     //4 //(D4)
#define ENA PB0     //3 //(D3)

//connections for right wheels motor
#define IN3 PC14    //7 //(D7)
#define IN4 PB1     //6 //(D6)
#define ENB PB6     //5 //(D5)

//could be defined as global - currently passed to functions
//int PWM_Value = 0;  //a value between 0 and 255 for PWM control

//function prototypes
void stopCar();
void forward(int);
void backward(int);
void left(int);
void right(int);


void setup() {
  // set L298N connections as outputs
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);
}

void loop() {
//forward(255);
//delay(1000);

//locomotion(270,500);

right(125);
delay(500.4);
forward(255);
delay(250);

//
//forward(125);
//delay(1000);
//
//right(125);
//delay(500);
//
//forward(125);
//delay(1000);
//
//right(125);
//delay(500);
//
//forward(125);
//delay(1000);
//
//right(125);
//delay(500);
//
//forward(125);
//delay(1000);
//
//right(125);
//delay(500);
//
//forward(125);
//delay(1000);
//
//left(125);
//delay(500);
//
//forward(125);
//delay(1000);
//
//left(125);
//delay(500);
//
//forward(125);
//delay(1000);
//
//left(125);
//delay(500);

stopCar();
delay(10000);
}
