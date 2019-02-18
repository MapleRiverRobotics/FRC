/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc7541.RocketBot.commands;

import org.usfirst.frc7541.RocketBot.Robot;

import edu.wpi.first.wpilibj.command.Command;

public class DriveStraightCommand extends Command {
    private double distance;

    public DriveStraightCommand(double distance) {
        this.distance = distance;
        requires(Robot.driveTrain);
    }

    // Called just before this Command runs the first time
    @Override
    protected void initialize() { 
        Robot.driveTrain.resetPosition();
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute() {
        if (distance > 0) {
            Robot.driveTrain.arcadeDrive(-0.5, 0);
        }
        else {
            Robot.driveTrain.arcadeDrive(0.5, 0);
        }
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    protected boolean isFinished() {
        //System.out.println(Robot.driveTrain.getPosition());

        if (distance > 0) {
            return Robot.driveTrain.getPosition() >= distance;
        }
        else {
            return Robot.driveTrain.getPosition() <= distance;
        }
    }

    // Called once after isFinished returns true
    @Override
    protected void end() {
        Robot.driveTrain.stop();
    }

}
