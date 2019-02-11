/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc7541.RocketBot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import org.usfirst.frc7541.RocketBot.RobotMap;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.command.PIDSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Rotator extends PIDSubsystem {

    Encoder rotatorControllerEncoder = 
        new Encoder(RobotMap.rotatorDIOChannel1, RobotMap.rotatorDIOChannel2, false, EncodingType.k4X);

    WPI_VictorSPX rotatorController = new WPI_VictorSPX(RobotMap.rotatorMotor);

    public Rotator() {
        super("Rotator", .01, 0.0025, .15);
        // Use these to get going:
        // setSetpoint() - Sets where the PID controller should move the system
        // to
        // enable() - Enables the PID controller.
        setAbsoluteTolerance(.5);
        setOutputRange(-.5, .5);

        // Original value: 0.6542480690595184
        rotatorControllerEncoder.setDistancePerPulse(0.622);
    }

    @Override
    protected double returnPIDInput() {
        // Return your input value for the PID loop
        double distance = rotatorControllerEncoder.getDistance();
        SmartDashboard.putNumber("Rotator", distance);
        return distance;
    }

    @Override
    protected void usePIDOutput(double output) {
        rotatorController.set(minimumSpeed(output));
    }


    @Override
    protected void initDefaultCommand() {
    }
    
    public void degreesToRotate(double degrees) {
        double kP = SmartDashboard.getNumber("P_Rotate", 0.05);
        double kI = SmartDashboard.getNumber("I_Rotate", 0.0025);
        double kD = SmartDashboard.getNumber("D_Rotate", 0.15);

        SmartDashboard.putNumber("P_Rotate", kP);
        SmartDashboard.putNumber("I_Rotate", kI);
        SmartDashboard.putNumber("D_Rotate", kD);

        getPIDController().setPID(kP, kI, kD);
        
        rotatorControllerEncoder.reset();
        setSetpoint(degrees);
    }

    public void stop() {
        rotatorController.set(0);
        disable();
        SmartDashboard.putNumber("Rotator", rotatorControllerEncoder.getDistance());
    }

    private double minimumSpeed(double d) {
        double minSpeed = 0.025;
        if (d> 0 && d < minSpeed) {
            d = minSpeed;
        } else if (d < 0 && d > -minSpeed) {
            d = -minSpeed;
        }
        return d;
    }

}