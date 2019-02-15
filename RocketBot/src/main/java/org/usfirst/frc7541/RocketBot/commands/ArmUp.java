package org.usfirst.frc7541.RocketBot.commands;

import edu.wpi.first.wpilibj.command.Command;
import org.usfirst.frc7541.RocketBot.Robot;

/**
 *
 */
public class ArmUp extends Command {

    public ArmUp() {
        requires(Robot.arm);
    }

    // Called just before this Command runs the first time
    @Override
    protected void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute() {
        Robot.arm.up();
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    protected boolean isFinished() {
        return Robot.arm.armIsRaised();
    }

    // Called once after isFinished returns true
    @Override
    protected void end() {
        Robot.arm.stop();

    }

}
