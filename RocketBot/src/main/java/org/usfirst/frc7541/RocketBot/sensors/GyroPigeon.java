/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc7541.RocketBot.sensors;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.wpilibj.SendableBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;

/**
 * This class abstracts the CTRE Pigeon Gyro
 * 
 * For more on the sensor, see this website:
 * http://www.ctr-electronics.com/sensors/gadgeteer-imu-module-pigeon.html#product_tabs_technical_resources
 */
public class GyroPigeon extends PigeonIMU {

    public GyroPigeon(int deviceNumber) {
        super(deviceNumber);
    }

    public GyroPigeon(WPI_TalonSRX talonSrx) {
        super(talonSrx);
    }

    /**
     * Get the robot's heading.
     *
     * @return The robots heading in degrees.
     */
    public double getHeading() {
        // double[] ypr = new double[3];
        // getYawPitchRoll(ypr);
        // return ypr[0];
        double heading = getFusedHeading();
        if (heading >= 360) {
            heading = heading % 360; // keep the remainder or modulas
        }
        return heading;
    }

    public void resetToZeroHeading() {
        setFusedHeading(0);
    }

}
