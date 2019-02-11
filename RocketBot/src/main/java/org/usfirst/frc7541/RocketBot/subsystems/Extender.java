package org.usfirst.frc7541.RocketBot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import org.usfirst.frc7541.RocketBot.RobotMap;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class Extender extends Subsystem {

    WPI_VictorSPX extenderController = new WPI_VictorSPX(RobotMap.extenderMotor);

    public Extender() {
    }

    @Override
    public void initDefaultCommand() {
    }

    @Override
    public void periodic() {
    }

    public void extend() {
        extenderController.set(1);
    }

    public void retract() {
        extenderController.set(-1);
    }

    public void stop() {
        extenderController.stopMotor();
    }
}
