package org.usfirst.frc7541.RocketBot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import org.usfirst.frc7541.RocketBot.RobotMap;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class Arm extends Subsystem {

    WPI_VictorSPX armController = new WPI_VictorSPX(RobotMap.armMotor);

    public Arm() {
    }

    @Override
    public void initDefaultCommand() {
    }

    @Override
    public void periodic() {
        // Put code here to be run every loop
    }
    
    public void up() {
        armController.set(0.4);
    }

    public void down() {
        armController.set(-0.4);
    }

    public void stop() {
        armController.stopMotor();
    }
}

