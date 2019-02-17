package org.usfirst.frc7541.RocketBot.subsystems;

import java.awt.Robot;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import org.usfirst.frc7541.RocketBot.RobotMap;
import org.usfirst.frc7541.RocketBot.commands.ExtenderCommand;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class Extender extends Subsystem {

    WPI_VictorSPX extenderController = new WPI_VictorSPX(RobotMap.extenderMotor);
    DigitalInput retractLimitSwitch = new DigitalInput(RobotMap.extenderRetractLimitSwitchDio);

    public Extender() {
    }

    @Override
    public void initDefaultCommand() {
        setDefaultCommand(new ExtenderCommand());
    }

    @Override
    public void periodic() {
    }

    public void extendSpeed(double speed){
        extenderController.set(speed);
    }
    // public void extend() {
    //     extenderController.set(1);
    // }

    // public void retract() {
    //     extenderController.set(-1);
    // }

    public void stop() {
        extenderController.stopMotor();
    }

    public boolean isRetracted() {
        return retractLimitSwitch.get() == false;
    }
}
