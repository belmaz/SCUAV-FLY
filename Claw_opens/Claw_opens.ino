// Include the Servo library 
#include <Servo.h> 

Servo myservo; 
int x = 0;
int servo_position=0;

void setup() { 
  Serial.print("Please Work!!!");
  // We need to attach the servo to the used pin number 
   myservo.attach(6); //attaches the servo on pin 6 to the servo object
   Serial.begin(9600);
   //x = 1;
   // want x = 1

}



void loop(){
  if(Serial.available())
  {
    x = Serial.read(); //read serial port    
    x = x - '0';
    
    for(int i = 0)
    
   if(x == 1);
  {
      // Servo needs to open at 45 degrees:
      for (servo_position=0; servo_position<=60; servo_position +=1)
    {
      myservo.write(servo_position);
      delay(5); //or if styrofoam falling out, make delay longer
      }
  exit (0);
  }
  exit(0);
 }
 exit(0);
}

/* NOTES:
  works with out the serial command thing 
    -currently being weird when add serial read and it isnt even connected to the pi yet
    -idk what is going on :/



*/

