/*
ARDUINO NANO SERVO CONTROLLER

The program recieves servo angles from ROS2 node 'motor_controller.py' over serial communication to move the MG90D motor using PWM timing pulses.

Standard Arduino Servo library uses range 0 to 180; where 0 is the min position and 180 is the max position of the servo.
ROS2 control node accepts input within range -135 to 135 deg which aligns with the actual range of MG90D motor.

ROS2 node converts incoming input commands (-135-135 deg) and maps to 0-180 range which is then used by the arduino.

Pin placements:
Brown wire - GND
Orange wire - D9
Red wire - 5V
*/
#include <Servo.h>

Servo panServo; // Global Servo object used to control MG90D
String incoming = ""; //Stores incoming serial characters until newline '\n' is recieved

// Start-up
void setup() {
    Serial.begin(115200); //Baud rate
    panServo.attach(9); //Signal pin - D9
    panServo.write(90); //Move servo to center position (90 within 0-180 range)
    Serial.println("READY");
}

// Runs continuously after setup()
void loop() {
    while (Serial.available()) {
        char c = Serial.read(); //Reads a character from the serial buffer
        
        if (c == '\n') { //Newline indicates end of command
            incoming.trim();  // Remove any whitespace or \r
            
            if (incoming.length() > 0) {  // Only parse if non-empty command is recieved
                int cmd = incoming.toInt(); //Convert incoming text to int command value
                cmd = constrain(cmd, 0, 180); //Clamp angle to valid servo range
                panServo.write(cmd); //Move to target angle by generating PWM pulse
                Serial.print("MOVING TO: ");
                Serial.println(cmd);
            }
            incoming = "";  // Always clear after recieving command
        }
        else if (c == '\r') {
            // Ignore carriage return completely
        }
        else { // Add normal character to command buffer
            incoming += c;
        }
    }
}
