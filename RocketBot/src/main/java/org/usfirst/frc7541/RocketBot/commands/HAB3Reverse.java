/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc7541.RocketBot.commands;

import org.usfirst.frc7541.RocketBot.Robot;

import edu.wpi.first.wpilibj.command.Command;

public class HAB3Reverse extends Command {
    public HAB3Reverse() {
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
        if(Robot.driveTrain.getPosition() < 10){
            Robot.driveTrain.arcadeDrive(-0.5,0);
        }else{
            Robot.driveTrain.arcadeDrive(0,0);
        }
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    protected boolean isFinished() {
        if (Robot.driveTrain.getPosition() >= 10) {
            return true;
        }
        else {
            return false;
        }
    }

    // Called once after isFinished returns true
    @Override
    protected void end() {
        Robot.driveTrain.stop();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    @Override
    protected void interrupted() {
    }
}
