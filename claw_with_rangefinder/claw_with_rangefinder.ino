#include <Servo.h>                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                

// Variables:
Servo myservo;
int angle = 0;
int val;
#define trigPin A3
#define echoPin 7


// Initial setup:
void setup(){
  Serial.begin(9600); // Set the baudrate
  myservo.attach(6); // Plug into pin 6
  val = 0;
    Serial.begin(9600);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  myservo.attach(6); //attaches the servo on pin 6 to the servo object
 
}


void loop()
{
    // put your main code here, to run repeatedly:
long duration, distance;
  digitalWrite(trigPin, LOW);  // Added this line
  delayMicroseconds(2); // Added this line
  digitalWrite(trigPin, HIGH);
//  delayMicroseconds(1000); - Removed this line
  delayMicroseconds(10); // Added this line
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  distance = (duration/2) / 29.1;

  Serial.print(distance);
    Serial.println(" cm");
 
  delay(500);

 // If the digit is 1, then close the claw:
    if(distance<10){
      close_claw();
    }
    
    // If the digit is 0, then open the claw:
    if(distance>10){
      open_claw();
    }
  }

// Close claw function: 
void close_claw(){
  angle = 100; // 100 degrees seems to close claw in a secure way
  myservo.write(angle); // send angle to the servo
  delay(1); 
}

// Open claw function:
void open_claw(){
  angle = 15; // 15 degrees seems to open the claw wide enough for travelling purposes
  myservo.write(angle); // send angle to the servo
  delay(1); 
}




