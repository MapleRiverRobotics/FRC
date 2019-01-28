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
    private static final int MAX_BYTES = 32;
    long[] lightValues = new long[5];
    
    public String read() {
        byte[] data = new byte[MAX_BYTES];// create a byte array to hold the incoming data
        Wire.read(I2C_DEVICE_ID, MAX_BYTES, data);// use address 4 on i2c and store it in data
        String output = new String(data);// create a string from the byte array
        int pt = output.indexOf((char) 255);
        return (String) output.subSequence(0, pt < 0 ? 0 : pt);// im not sure what these last two lines do
    }

    public static int byteArrayToLeInt(byte[] b) {
        final ByteBuffer bb = ByteBuffer.wrap(b);
        bb.order(ByteOrder.LITTLE_ENDIAN);
        return bb.getInt();
    }

}
