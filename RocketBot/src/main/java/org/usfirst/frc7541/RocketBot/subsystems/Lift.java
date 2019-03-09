package org.usfirst.frc7541.RocketBot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import org.usfirst.frc7541.RocketBot.RobotMap;

import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * This class raises and lowers the hatch panel carrying arm on the robot
 */
public class Lift extends Subsystem {

    WPI_VictorSPX liftController = new WPI_VictorSPX(RobotMap.liftMotor);

    public Lift() {
    }

    @Override
    public void initDefaultCommand() {
    }

    public void up() {
        liftController.set(RobotMap.liftSpeed);

    }

    public void down() {
        liftController.set(-RobotMap.liftSpeed);
    }

    public void stop() {
        liftController.stopMotor();
    }
}
