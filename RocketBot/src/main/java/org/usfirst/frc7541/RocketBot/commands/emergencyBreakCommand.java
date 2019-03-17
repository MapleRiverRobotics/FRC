/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc7541.RocketBot.commands;

import org.usfirst.frc7541.RocketBot.Robot;

import edu.wpi.first.wpilibj.command.Command;

public class emergencyBreakCommand extends Command {
  public emergencyBreakCommand() {
    requires(Robot.arm);
    requires(Robot.driverRotator);
    requires(Robot.driveTrain);
    requires(Robot.extender);
    requires(Robot.lift);
    requires(Robot.rotator);

  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
      Robot.arm.stop();
      Robot.driverRotator.stop();
      Robot.driveTrain.stop();
      Robot.extender.stop();
      Robot.lift.stop();
      Robot.rotator.stop();
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return true;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
