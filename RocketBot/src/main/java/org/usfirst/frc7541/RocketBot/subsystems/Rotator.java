/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc7541.RocketBot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import org.usfirst.frc7541.RocketBot.Robot;
import org.usfirst.frc7541.RocketBot.commands.RotatorRotateLeft;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.command.PIDSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Rotator extends PIDSubsystem {

    Encoder armControllerEncoder;

    WPI_VictorSPX armController = new WPI_VictorSPX(5);

    Joystick gamepad1 = new Joystick(0);

    public Rotator() {
        super("Rotator", 1, 0, 1);
        double kP = SmartDashboard.getNumber("P_Rotate", 0.05);
        double kI = SmartDashboard.getNumber("I_Rotate", 0.0025);
        double kD = SmartDashboard.getNumber("D_Rotate", 0.15);

        SmartDashboard.putNumber("P_Rotate", kP);
        SmartDashboard.putNumber("I_Rotate", kI);
        SmartDashboard.putNumber("D_Rotate", kD);

        getPIDController().setPID(kP, kI, kD);
        // Use these to get going:
        // setSetpoint() - Sets where the PID controller should move the system
        // to
        // enable() - Enables the PID controller.
        setAbsoluteTolerance(1);
    }

    @Override
    public void initDefaultCommand() {
        armControllerEncoder = new Encoder(1, 2, false, EncodingType.k4X);
        armControllerEncoder.setDistancePerPulse(0.6542480690595184);
    }

    @Override
    protected double returnPIDInput() {
        // Return your input value for the PID loop
        // e.g. a sensor, like a potentiometer:
        // yourPot.getAverageVoltage() / kYourMaxVoltage;
        double distance = armControllerEncoder.getDistance();
        SmartDashboard.putNumber("Rotation", distance);
        return distance;
    }

    @Override
    protected void usePIDOutput(double output) {
        armController.set(output);
    }

    public void degreesToRotate(double degrees) {
        setSetpoint(armControllerEncoder.getDistance() + degrees);
    }

}