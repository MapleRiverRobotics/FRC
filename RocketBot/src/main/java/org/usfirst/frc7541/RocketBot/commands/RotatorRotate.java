// RobotBuilder Version: 2.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.

package org.usfirst.frc7541.RocketBot.commands;

import edu.wpi.first.wpilibj.command.Command;
import org.usfirst.frc7541.RocketBot.Robot;

/**
 *
 */
public class RotatorRotate extends Command {

    private double _degrees = 0;
    public RotatorRotate(double degrees) {
        requires(Robot.rotator);
        _degrees = degrees;
    }

    // Called just before this Command runs the first time
    @Override
    protected void initialize() {
        Robot.rotator.degreesToRotate(_degrees);
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute() {
        Robot.rotator.enable();
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    protected boolean isFinished() {
        return Robot.rotator.onTarget();
    }

    // Called once after isFinished returns true
    @Override
    protected void end() {
        Robot.rotator.stop();
    }

}
