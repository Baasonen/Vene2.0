#include "magnetometer.h"

Adafruit_BNO08x bno085;
sh2_SensorValue_t sensorValue;

int magInit()
{
    if (!bno085.begin_I2C())
    {
        Serial.println("BNO085 not found");
        return 0;
    }

    if (!bno085.enableReport(SH2_ARVR_STABILIZED_ROTATION_VECTOR, 10000)) // 100 Hz
    {
        Serial.println("Could not enable stablized rotation vector");
        return 0;
    }

    return 1;
}

MagData getMagnetometer() 
{   
    static uint32_t lastValidTime;
    static MagData lastValid = {};

    if (bno085.getSensorEvent(&sensorValue)) {
        if (sensorValue.sensorId == SH2_ARVR_STABILIZED_ROTATION_VECTOR) {
            
            float r = sensorValue.un.rotationVector.real;
            float i = sensorValue.un.rotationVector.i;
            float j = sensorValue.un.rotationVector.j;
            float k = sensorValue.un.rotationVector.k;

            float siny_cosp = 2.0 * (r * k + i * j);
            float cosy_cosp = 1.0 - 2.0 * (j * j + k * k);
            float heading = atan2(siny_cosp, cosy_cosp);

            heading *= 180.0 / PI;

            if (heading < 0) heading += 360.0;
            
            heading = 360.0 - heading; 
            uint8_t accuracy = sensorValue.status & 0x03;

            
            lastValid.heading = heading;
            lastValid.accuracy = accuracy;
            lastValid.valid = true;

            lastValidTime = millis();
        }
    }
    
    if ((millis() - lastValidTime) > 200) {lastValid.valid = false;}

    return lastValid;
}