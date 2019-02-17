package org.usfirst.frc7541.RocketBot.commands;

import edu.wpi.first.wpilibj.command.Command;
import org.usfirst.frc7541.RocketBot.Robot;

/**
 *
 */
public class ExtenderRetractCommand extends Command {

    public ExtenderRetractCommand() {
        requires(Robot.extender);
    }

    // Called just before this Command runs the first time
    @Override
    protected void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute() {
        //Robot.extender.retract();
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    protected boolean isFinished() {
        return Robot.extender.isRetracted();
    }

    // Called once after isFinished returns true
    @Override
    protected void end() {
        Robot.extender.stop();
    }
}
