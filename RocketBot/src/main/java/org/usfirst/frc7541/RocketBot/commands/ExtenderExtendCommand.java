package org.usfirst.frc7541.RocketBot.commands;

import edu.wpi.first.wpilibj.command.Command;
import org.usfirst.frc7541.RocketBot.Robot;

/**
 *
 */
public class ExtenderExtendCommand extends Command {

    public ExtenderExtendCommand() {
         requires(Robot.extender);
    }

    // Called just before this Command runs the first time
    @Override
    protected void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute() {
        //Robot.extender.extend();
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    @Override
    protected void end() {
        Robot.extender.stop();
    }
}
