package org.usfirst.frc7541.RocketBot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import org.usfirst.frc7541.RocketBot.RobotMap;

public class DriverRotator extends Subsystem {

    WPI_VictorSPX rotatorController = new WPI_VictorSPX(RobotMap.rotatorMotor);

    public DriverRotator() {
    }

    @Override
    public void initDefaultCommand() {
    }

    @Override
    public void periodic() {
        // Put code here to be run every loop
    }

    public void right() {
        rotatorController.set(0.5);
    }

    public void left() {
        rotatorController.set(-0.5);
    }

    public void stop() {
        rotatorController.set(0);
    }
}
