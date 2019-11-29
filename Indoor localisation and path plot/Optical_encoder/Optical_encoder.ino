

// Include the TimerOne Library from Paul Stoffregen
#include "TimerOne.h"

float x=0;//initial abcissa value
float y=0;//initial ordinate value
float p=3.14;

const byte MOTOR1 = 2;  // Motor 1 Interrupt Pin - INT 0
const byte MOTOR2 = 3;  // Motor 2 Interrupt Pin - INT 1

// Integers for pulse counters
unsigned int counter1 = 0;
unsigned int counter2 = 0;

// Float for number of slots in encoder disk
float diskslots = 80;  // Change to match value of encoder disk
float radius=3.0;//radius of wheels in meters
float dist=13.0;//distance between the wheels in meters
float s_th=0;

// Motor 1 pulse count ISR
void ISR_count1()  
{
  counter1++;  // increment Motor 1 counter value
} 

// Motor 2 pulse count ISR
void ISR_count2()  
{
  counter2++;  // increment Motor 2 counter value
} 
int distance(int a,int b)
{

float v1=(a)*2*p*radius/60;
float v2=(b)*2*p*radius/60;
float th=(v1-v2)/dist;
s_th+=th;
if((int)abs(s_th*100)%628<150)   //when the vehicle is in first co-ordinate
{
  if(v1>v2){
    x=x-v1*sin(th);
    y=y+v1*cos(th);
  }
  else
  {
    x=x+v2*sin(th);
    y=y+v2*cos(th);
  }
}
else if((int)abs(s_th*100)%628>150 && (int)abs(s_th*100)%628<310)  //when the vehicle is in second co-ordinate
{
  if(v1>v2)
  {
    y=y+v1*sin(th);
    x=x-v1*cos(th);
   }
  else
  {
    y=y+v2*sin(th);
    x=x-v2*cos(th);
  }
}
else if((int)abs(s_th*100)%628>310 && (int)abs(s_th*100)%628<470)    //when the vehicle is in third co-ordinate
{
  if(v1>v2){
    x=x-v1*sin(th);
    y=y-v1*cos(th);
  }
  else
  {
    x=x+v2*sin(th);
    y=y-v2*cos(th);
  }
}
else
{
  if(v1>v2){
    y=y+v1*sin(th);
    x=x-v1*cos(th);
  }
  else
  {
    y=y-v2*sin(th);
    x=x+v2*cos(th);
  }
}
Serial.print("Ordinate of the vehicle : ");
Serial.print(y);
Serial.print("   "); 
Serial.print("Abcissa of the vehicle : ");
Serial.println(x);
}
int motor1()
{
  int res;
    Timer1.detachInterrupt();  // Stop the timer
    //Serial.print("Motor Speed 1: "); 
    float rotation1 = (counter1 / diskslots) * 60 ;  // calculate RPM for Motor 1 
    res=(int)rotation1;
    counter1 = 0;  //  reset counter to zero
    Timer1.attachInterrupt( ISR_timerone );  // Enable the timer
    return res ;

  }

  int motor2()
  {
    int res;
    Timer1.detachInterrupt();  // Stop the timer
    //Serial.print("Motor Speed 2: "); 
    float rotation2 = (counter2 / diskslots) * 60;  // calculate RPM for Motor 2
    res=(int)rotation2;
    counter2 = 0;  //  reset counter to zero
    Timer1.attachInterrupt( ISR_timerone );  // Enable the timer
    return res;
  }
        // TimerOne ISR
void ISR_timerone()
{
  //motor1();
  //motor2();
  distance(motor1(),motor2());
  delay(1000);
}

void setup() 
{
  Serial.begin(9600);

 // motor 1 consists of motorpin3, and motorpin4
 // motor 2 consists of motorpin1, and motorpin2

    const int ena=11;
    const int enb=12;
    const int motorPin1  = 7;  // Pin 14 of L293
    const int motorPin2  = 8;  // Pin 10 of L293//Motor B
    const int motorPin3  = 5; // Pin  7 of L293
    const int motorPin4  = 6;  // Pin  2 of L293
    pinMode(motorPin1, OUTPUT);
    pinMode(motorPin2, OUTPUT);
    pinMode(motorPin3, OUTPUT);
    pinMode(motorPin4, OUTPUT);
    pinMode(ena, OUTPUT);
    pinMode(enb, OUTPUT);
 
    digitalWrite(motorPin1, LOW);
    digitalWrite(motorPin2, HIGH);
    digitalWrite(motorPin3, LOW);
    digitalWrite(motorPin4, HIGH);

    delay(1000);

  Timer1.initialize(1000000); // set timer for 1sec
  attachInterrupt(digitalPinToInterrupt (MOTOR1), ISR_count1, RISING);  // Increase counter 1 when speed sensor pin goes High
  attachInterrupt(digitalPinToInterrupt (MOTOR2), ISR_count2, RISING);  // Increase counter 2 when speed sensor pin goes High
  Timer1.attachInterrupt( ISR_timerone ); // Enable the timer
} 

void loop()
{
  // Nothing in the loop!
  // You can place code here

}
