#include <QTRSensors.h>
#include <Wire.h>

#define I2C_DEVICE_ID 4
#define UNSIGNED_INT_SIZE 2
#define NUM_SENSORS 2                  // number of sensors used
#define NUM_SAMPLES_PER_SENSOR 4       // average 4 analog samples per sensor reading
#define EMITTER_PIN QTR_NO_EMITTER_PIN // emitter is controlled by digital pin 2

// sensors 0 through 4 are connected to analog inputs 0 through 4, respectively
QTRSensorsAnalog qtra((unsigned char[]){0, 1}, NUM_SENSORS, NUM_SAMPLES_PER_SENSOR, EMITTER_PIN);

static unsigned int sensorValues[NUM_SENSORS] = {0,0};

void setup()
{
    delay(500);
    Serial.begin(9600); // set the data rate in bits per second for serial data transmission
    delay(1000);
    Wire.begin(I2C_DEVICE_ID);    // join i2c bus with address #4
    Wire.setClock(400000); // set clock speed to fast to match RoboRio I2C clock speed
    Wire.onRequest(requestEvent); // register event
}

void loop()
{
    delay(20);

    // read raw sensor values - 0 (maximum reflectance) to 1023 (minimum reflectance)
    readSensors();
    requestEvent();
}

void readSensors() 
{
    unsigned int tempValues[NUM_SENSORS];
    qtra.read(tempValues);

    noInterrupts();
    memcpy(sensorValues, tempValues, sizeof(tempValues));
    interrupts();
}

// function that executes whenever data is requested by master
// this function is registered as an event, see setup()
void requestEvent()
{
    byte lightValues[NUM_SENSORS * UNSIGNED_INT_SIZE];

    // print the sensor values as numbers from 0 to 1023, where 0 means maximum reflectance and
    // 1023 means minimum reflectance
    for (int i = 0; i < NUM_SENSORS; i++)
    {
        appendToByteArray(i, sensorValues[i], lightValues);
        Serial.print(sensorValues[i]);
        Serial.print('\t'); // tab to format the raw data into columns in the Serial monitor
    }
    Serial.println();

    Wire.write(lightValues, (NUM_SENSORS * UNSIGNED_INT_SIZE));
}

void appendToByteArray(unsigned char arrayIndex, unsigned int lightValue, byte lightValues[])
{
    byte *bytePointer = (byte *)&lightValue;
    unsigned char position = 0;

    for (int i = UNSIGNED_INT_SIZE - 1; i > -1; i--)
    {
        lightValues[(arrayIndex * UNSIGNED_INT_SIZE) + position] = bytePointer[i];
        position++;
    }
}
