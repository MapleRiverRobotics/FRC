package org.usfirst.frc7541.RocketBot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import org.usfirst.frc7541.RocketBot.RobotMap;

import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * This class raises and lowers the hatch panel carrying arm on the robot
 */
public class Arm extends Subsystem {

    WPI_VictorSPX armController = new WPI_VictorSPX(RobotMap.armMotor);
    DigitalInput lowerLimitSwitch = new DigitalInput(RobotMap.armLowerLimitSwitchDio);
    DigitalInput raiseLimitSwitch = new DigitalInput(RobotMap.armRaiseLimitSwitchDio);

    public Arm() {
    }

    @Override
    public void initDefaultCommand() {
    }

    public void up() {
        armController.set(RobotMap.armSpeed);

    }

    public void down() {
        armController.set(-RobotMap.armSpeed);
    }

    public boolean armIsRaised() {
        return raiseLimitSwitch.get() == false;
    }

    public boolean armIsLowered() {
        return lowerLimitSwitch.get() == false;
    }

    public void stop() {
        armController.stopMotor();
    }
}
