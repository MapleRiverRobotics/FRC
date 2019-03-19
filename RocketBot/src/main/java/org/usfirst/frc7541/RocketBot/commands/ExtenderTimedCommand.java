/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc7541.RocketBot.commands;

import org.usfirst.frc7541.RocketBot.Robot;

import edu.wpi.first.wpilibj.command.TimedCommand;

public class ExtenderTimedCommand extends TimedCommand {
  public ExtenderTimedCommand(double timeout) {
      super(timeout);
      requires(Robot.extender);
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    Robot.extender.extendSpeed(0.5);
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.extender.extendSpeed(0);
  }
}
