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

    private static final int I2C_DEVICE_ID = 4;
    public I2C Wire = new I2C(Port.kOnboard, I2C_DEVICE_ID);
    private static final int BYTES_PER_SENSOR = 2;
    private static final int COUNT_OF_SENSORS = 5;
    private static final int TOTAL_BYTES = BYTES_PER_SENSOR * COUNT_OF_SENSORS;

    public String read() {
        byte[] data = new byte[TOTAL_BYTES];// create a byte array to hold the incoming data
        Wire.read(I2C_DEVICE_ID, TOTAL_BYTES, data);// use address 4 on i2c and store it in data

        int[] numbers = byteArrayToIntArray(data);

        for (int i = 0; i < COUNT_OF_SENSORS; i++) {
            System.out.print(numbers[i]);
            System.out.print("  ");
        }
        System.out.println();
        return null;
    }

    public static int[] byteArrayToIntArray(byte[] b) {
        final ByteBuffer bb = ByteBuffer.wrap(b);
        bb.order(ByteOrder.BIG_ENDIAN);

        int[] results = new int[COUNT_OF_SENSORS];
        for (int i = 0; i < COUNT_OF_SENSORS; i++) {
            results[i] = bb.getShort();
        }
        return results;
    }

}
