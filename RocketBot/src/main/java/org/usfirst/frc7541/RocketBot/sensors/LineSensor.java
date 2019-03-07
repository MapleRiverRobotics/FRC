/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc7541.RocketBot.sensors;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.I2C.Port;

public class LineSensor {

    private static final short I2C_DEVICE_ID = 4;
    public I2C Wire = new I2C(Port.kOnboard, I2C_DEVICE_ID);
    private static final short BYTES_PER_SENSOR = 2;
    private static final short COUNT_OF_SENSORS = 2;
    private static final short TOTAL_BYTES = BYTES_PER_SENSOR * COUNT_OF_SENSORS;

    private static final short SIDE_LEFT_SENSOR = 0;
    private static final short SIDE_RIGHT_SENSOR = 1;
    // private static final short FRONT_LEFT_SENSOR = 0;
    // private static final short FRONT_CENTER_SENSOR = 1;
    // private static final short FRONT_RIGHT_SENSOR = 2;

    private static final short LINE_VISIBLE_VALUE = 700; // when the sensor reads less than 600, we are over a white line
    private static final short LINE_NOT_VISIBLE_VALUE = 800; // when the sensor reads less than 600, we are over a white line

    private static short[] onLineValues;
    private static short[] offLineValues;

    public enum LineSensorStatus {
        NoReading, LeftOfLine, Centered, RightOfLine, NotOnLine, Indeterminent
    }

    // static variable single_instance so we only have one of these objects in memory
    private static LineSensor single_instance = null;

    // private constructor restricted to this class so that we only initialize once
    private LineSensor() {
        onLineValues = new short[COUNT_OF_SENSORS];
        offLineValues = new short[COUNT_OF_SENSORS];
        for (short i = 0; i < COUNT_OF_SENSORS; i++) {
            onLineValues[i] = LINE_VISIBLE_VALUE;
            offLineValues[i] = LINE_NOT_VISIBLE_VALUE;
        }
    }

    // static method to create instance of Singleton class
    public static LineSensor getInstance() {
        if (single_instance == null)
            single_instance = new LineSensor();

        return single_instance;
    }

    public LineSensorStatus leftSideLineStatus(double percentErrorAllowed) {
        short[] sensorValues = readSensorValues();
        return getLineSensorStatus(SIDE_LEFT_SENSOR, percentErrorAllowed, sensorValues);
    }

    public LineSensorStatus rightSideLineStatus(double percentErrorAllowed) {
        short[] sensorValues = readSensorValues();
        return getLineSensorStatus(SIDE_RIGHT_SENSOR, percentErrorAllowed, sensorValues);
    }

    // public LineSensorStatus frontLineStatus() {
    //     short[] sensorValues = readSensorValues();

    //     if (sensorValues[FRONT_LEFT_SENSOR] <= 0 && sensorValues[FRONT_CENTER_SENSOR] <= 0
    //             && sensorValues[FRONT_RIGHT_SENSOR] <= 0) {
    //         return LineSensorStatus.NoReading;
    //     }
    //     if (sensorValues[FRONT_LEFT_SENSOR] < 600) {
    //         return LineSensorStatus.RightOfLine;
    //     }
    //     if (sensorValues[FRONT_RIGHT_SENSOR] < 600) {
    //         return LineSensorStatus.LeftOfLine;
    //     }
    //     if (sensorValues[FRONT_CENTER_SENSOR] < 600) {
    //         return LineSensorStatus.Centered;
    //     }
    //     return LineSensorStatus.NotOnLine;
    // }

    private LineSensorStatus getLineSensorStatus(short sensorIndex, double percentErrorAllowed, short[] sensorValues) {

        short sensorValue = sensorValues[sensorIndex];

        // store sensor values for future use
        if (sensorValue < onLineValues[sensorIndex])
            onLineValues[sensorIndex] = sensorValue;
        if (sensorValue > offLineValues[sensorIndex])
            offLineValues[sensorIndex] = sensorValue;

        if (sensorValue <= 0)
            return LineSensorStatus.NoReading;

        if (sensorValue < onLineValues[sensorIndex] * ((100 + percentErrorAllowed) / 100))
            return LineSensorStatus.Centered;

        if (sensorValue >= offLineValues[sensorIndex] * ((100 - percentErrorAllowed) / 100))
            return LineSensorStatus.NotOnLine;

        return LineSensorStatus.Indeterminent;
    }

    private short[] readSensorValues() {
        byte[] sensorData = new byte[TOTAL_BYTES]; // create a byte array to hold the incoming data
        Wire.readOnly(sensorData, TOTAL_BYTES);
        //Wire.read(I2C_DEVICE_ID, TOTAL_BYTES, sensorData); // read the data from the Arduino

        short[] sensorValues = byteArrayToIntArray(sensorData);

        for (int i = 0; i < COUNT_OF_SENSORS; i++) {
            System.out.print(sensorValues[i]);
            System.out.print("  ");
        }
        System.out.println();
        return sensorValues;
    }

    private static short[] byteArrayToIntArray(byte[] b) {
        final ByteBuffer bb = ByteBuffer.wrap(b);
        bb.order(ByteOrder.BIG_ENDIAN);

        short[] results = new short[COUNT_OF_SENSORS];
        for (int i = 0; i < COUNT_OF_SENSORS; i++) {
            results[i] = bb.getShort();
        }
        return results;
    }

}
