/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc7541.RocketBot.sensors;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;

/**
 * This class abstracts the Maxbotix MB1013 Ultrasonic sensor which has a range
 * of 30cm to 5 meters. The sensor is an analog sensor and should be plugged
 * into one of the analog channels on the roboRio.
 * 
 * For specs on the sensor, see this datasheet:
 * https://www.maxbotix.com/documents/HRLV-MaxSonar-EZ_Datasheet.pdf
 */
public class UltrasonicMB1013 extends AnalogInput {

    private static final double SUPPLY_VOLTAGE = 5.0;
    private static final double MILLIMETER_TO_INCH_MULTIPLIER = 25.4;
    private static final double ANALOG_SCALE_FACTOR = 1024;

    public UltrasonicMB1013(int channel) {
        super(channel);
    }

    /**
     * Get the distance to an obstacle in inches from the back of the sensor PCB
     * board to the face of the obstacle.
     *
     * @return The distance to the obstacle detected by the ultransonic sensor.
     */
    public double getDistanceInInches() {
        double rangeMM = 5 * (getAverageVoltage() / (SUPPLY_VOLTAGE / ANALOG_SCALE_FACTOR));
        return rangeMM / MILLIMETER_TO_INCH_MULTIPLIER;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
      builder.setSmartDashboardType("Ultrasonic");
      builder.addDoubleProperty("Value", this::getDistanceInInches, null);
    }

}
