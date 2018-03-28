#include <Arduino.h>

#include "Fin.h"
#include "Config.h"
#include "PWMServo.h"

int pins[] = {3, 4, 5};
int offset[] = {0, 0, 0};

PWMServo servos[3];
Fin fin(3);
int counter = 0;
int count = 1;
int amp = 100;

void setup() {
    for(int i = 0; i < 3; i++){
        servos[i].attach(pins[i]);
    }
    // put your setup code here, to run once:
    pinMode(13, OUTPUT);
    digitalWrite(13, HIGH);
    fin.init(servos, offset);
    digitalWrite(13, LOW);
    delay(1000);
    digitalWrite(13, HIGH);

    Serial.begin(115200);

}

void loop() {
    counter += 1;
    fin.write(count * 100 + 100, LEFT_PROPEL_DIRECTION, 0, true);
    // put your main code here, to run repeatedly:
    delay(1000 * UPDATE_INTERVAL);
    if(counter % 200 == 0){
        count *= -1;
    }
}