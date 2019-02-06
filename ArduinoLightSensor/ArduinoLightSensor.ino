#include <QTRSensors.h>
#include <Wire.h>

#define I2C_DEVICE_ID 4
#define UNSIGNED_INT_SIZE 2
#define NUM_SENSORS 5                  // number of sensors used
#define NUM_SAMPLES_PER_SENSOR 4       // average 4 analog samples per sensor reading
#define EMITTER_PIN QTR_NO_EMITTER_PIN // emitter is controlled by digital pin 2

// sensors 0 through 4 are connected to analog inputs 0 through 4, respectively
QTRSensorsAnalog qtra((unsigned char[]){0, 1, 2, 3, 4}, NUM_SENSORS, NUM_SAMPLES_PER_SENSOR, EMITTER_PIN);
unsigned int sensorValues[NUM_SENSORS];

byte m_lightValues[NUM_SENSORS * UNSIGNED_INT_SIZE];
int m_arrayIndex;

void setup()
{
    delay(500);
    Serial.begin(9600); // set the data rate in bits per second for serial data transmission
    delay(1000);
    Wire.begin(I2C_DEVICE_ID);    // join i2c bus with address #4
    Wire.onRequest(requestEvent); // register event
}

void loop()
{
    delay(100);
}

// function that executes whenever data is requested by master
// this function is registered as an event, see setup()
void requestEvent()
{
    m_arrayIndex = 0;

    // read raw sensor values - 0 (maximum reflectance) to 1023 (minimum reflectance)
    qtra.read(sensorValues);

    // print the sensor values as numbers from 0 to 1023, where 0 means maximum reflectance and
    // 1023 means minimum reflectance
    for (unsigned char i = 0; i < NUM_SENSORS; i++)
    {
        appendToByteArray(sensorValues[i]);
        Serial.print(sensorValues[i]);
        Serial.print('\t'); // tab to format the raw data into columns in the Serial monitor
    }
    Serial.println();

    Wire.write(m_lightValues, sizeof(m_lightValues));
}

void appendToByteArray(unsigned int lightValue)
{
    byte *bytePointer = (byte *)&lightValue;

    for (int i = UNSIGNED_INT_SIZE - 1; i > -1; i--)
    {
        m_lightValues[m_arrayIndex] = bytePointer[i];
        m_arrayIndex++;
    }
}
