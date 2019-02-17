package org.usfirst.frc7541.RocketBot.commands;

import org.usfirst.frc7541.RocketBot.Robot;
import edu.wpi.first.wpilibj.command.Command;

public class ExtenderCommand extends Command {

    public ExtenderCommand() {
        requires(Robot.extender);
    }

    // Called just before this Command runs the first time
    @Override
    protected void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute() {
        double speed = Robot.oi.getJoystickOperatorExtenderSpeed();

        Robot.extender.extendSpeed(speed);
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    protected boolean isFinished() {
        double speed = Robot.oi.getJoystickOperatorExtenderSpeed();
        if (speed < 0) {
            return Robot.extender.isRetracted();
        }
        return false;
    }

    // Called once after isFinished returns true
    @Override
    protected void end() {
        Robot.extender.extendSpeed(0);
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    @Override
    protected void interrupted() {
    }
}
