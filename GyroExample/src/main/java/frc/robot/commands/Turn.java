/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.command.Command;

import frc.robot.Robot;

/**
 * Drive the given distance straight (negative values go backwards). Uses a
 * local PID controller to run a simple PID loop that is only enabled while this
 * command is running. The input is the averaged values of the left and right
 * encoders.
 */
public class Turn extends Command {
  private final PIDController m_pid;

  private double m_degrees;
  /**
   * Create a new DriveStraight command.
   * @param distance The distance to drive
   */
  public Turn(double degrees) {
    requires(Robot.m_drivetrain);

    m_pid = new PIDController(.1, 0, 0, new PIDSource() {
      PIDSourceType m_sourceType = PIDSourceType.kDisplacement;

      @Override
      public double pidGet() {
        return Robot.m_drivetrain.getHeading();
      }

      @Override
      public void setPIDSourceType(PIDSourceType pidSource) {
        m_sourceType = pidSource;
      }

      @Override
      public PIDSourceType getPIDSourceType() {
        return m_sourceType;
      }
    },
    d -> Robot.m_drivetrain.turnLeft(d));

    m_pid.setAbsoluteTolerance(.5);
    m_degrees = degrees;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    // Get everything in a safe starting state.
    Robot.m_drivetrain.reset();
    m_pid.reset();
    m_pid.setSetpoint(Robot.m_drivetrain.getHeading() - m_degrees);
    m_pid.enable();
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return m_pid.onTarget();
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    // Stop PID and the wheels
    m_pid.disable();
    Robot.m_drivetrain.drive(0, 0);
  }
}
