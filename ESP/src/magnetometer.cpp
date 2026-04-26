#include "magnetometer.h"

Adafruit_BNO08x bno085(-1);
sh2_SensorValue_t sensorValue;

static bool magInitialized = false;

int magInit()
{
    bno085.hardwareReset();
    delay(1000); // Give time for proper bootup before I2C

    Wire.end();
    delay(100);
    Wire.begin();
    delay(200);

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

    magInitialized = true;
    return 1;
}

MagData getMagnetometer() 
{   
    static uint32_t lastValidTime = millis();
    static MagData lastValid = {};

    if (!magInitialized)
    {
        lastValid.valid = false;

        if ((millis() - lastValidTime) > 15000)
        {
            Serial.println("[MAG] Attempting re-init...");
            if (magInit()) {lastValidTime = millis();}
            else {lastValidTime = millis();}
        }
        return lastValid;
    }

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

    if ((millis() - lastValidTime) > 4000)
    {
        Serial.println("[MAG] Stale data, attempting to re-init...");
        lastValid.valid = false;
        magInitialized = false;
    }

    return lastValid;
}