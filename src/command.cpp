#include <Arduino.h>

float desired_throttle = 0.0; // Initialize throttle value
float desired_rotate = 0.0;   // Initialize rotation value

void parseCommand(String command)
{
    command.trim(); // Remove any leading or trailing whitespace

    if (command.startsWith("move"))
    {
        // Example: move 0.5 0.2
        int firstSpace = command.indexOf(' ');
        int secondSpace = command.indexOf(' ', firstSpace + 1);

        if (firstSpace > 0 && secondSpace > firstSpace)
        {
            String throttleValue = command.substring(firstSpace + 1, secondSpace);
            String rotateValue = command.substring(secondSpace + 1);

            desired_throttle = constrain(throttleValue.toFloat(), -1.0, 1.0);
            desired_rotate = constrain(rotateValue.toFloat(), -1.0, 1.0);

            Serial.print("Throttle set to: ");
            Serial.println(desired_throttle);
            Serial.print("Rotation set to: ");
            Serial.println(desired_rotate);
        }
    }
    else if (command.equals("stop"))
    {
        desired_throttle = 0.0;
        desired_rotate = 0.0;
        Serial.println("Robot stopped.");
    }
    else
    {
        Serial.println("Unknown command.");
    }
}