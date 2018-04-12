#include <Servo.h>

// Variables:
Servo myservo;
int angle = 0;
int val;

// Initial setup:
void setup(){
  Serial.begin(9600); // Set the baudrate
  myservo.attach(6); // Plug into pin 6
  val = 0;
}

// Continuous loop: 
void loop(){
  while(Serial.available()){ // Searching for Serial command
    char ch = Serial.read(); // Read in the Serial chacter from Raspberry Pi
    Serial.write(ch); // Write in the chacter
    
    val = ch - '0'; // Convert the character to a digit
    
    // If the digit is 1, then close the claw:
    if(val == 1){
      close_claw();
    }
    
    // If the digit is 0, then open the claw:
    if(val == 0){
      open_claw();
    }
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
