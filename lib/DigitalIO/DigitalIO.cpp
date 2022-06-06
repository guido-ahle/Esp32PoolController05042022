#include "DigitalIO.h"

#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include <Arduino.h>

boolean result = false;
 

void DigitalIO::startSetup()
{
    // Init der digitalen IO's
    delayMicroseconds(1000);
    pinMode(SCHWALLWASSERPUMPE, OUTPUT);
    pinMode(FILTERPUMPE, OUTPUT);
    pinMode(UWS, OUTPUT);
    pinMode(WP, OUTPUT);

}

boolean setIO_On(int gpio)
{
    result = false;
    digitalWrite(gpio, HIGH);
    digitalWrite(gpio, LOW);
    delayMicroseconds(1000);
    if (digitalRead(gpio) == 1)
    {
        result = true;
    };
    return result;
}

boolean setIO_Off(int gpio)
{
    result = false;
    digitalWrite(gpio, LOW);
    delayMicroseconds(1000);
    if (digitalRead(gpio) == 0)
    {
        result = true;
    };
    return result;
}