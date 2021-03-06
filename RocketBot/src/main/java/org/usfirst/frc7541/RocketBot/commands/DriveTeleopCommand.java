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
public class DriveTeleopCommand extends Command {

    public DriveTeleopCommand() {
        requires(Robot.driveTrain);
    }

    // Called just before this Command runs the first time
    @Override
    protected void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute() {
        double throttleSpeed = Robot.oi.getJoystickDriveThrottleSpeed();
        double forwardSpeed = Robot.oi.getJoystickDriveForwardSpeed() * throttleSpeed;
        double rotation = Robot.oi.getJoystickDriveRotation() * throttleSpeed;
    
        // Call the drive method to move the robot
        Robot.driveTrain.arcadeDrive(forwardSpeed, rotation);
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    protected boolean isFinished() {
        return false; // Runs until interrupted
    }

    // Called once after isFinished returns true
    @Override
    protected void end() {
        Robot.driveTrain.arcadeDrive(0, 0);
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    @Override
    protected void interrupted() {
    }
}
