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
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

import frc.robot.Robot;

/**
 * Drive the given distance straight (negative values go backwards). Uses a
 * local PID controller to run a simple PID loop that is only enabled while this
 * command is running. The input is the averaged values of the left and right
 * encoders.
 */
public class DriveStraight extends Command {
  private final PIDController m_pid;
  private Timer m_timer = new Timer();
  private double m_distance;
  /**  
   * Create a new DriveStraight command.
   * @param distance The distance to drive
   */
  public DriveStraight(double distance) {
    requires(Robot.m_drivetrain);

    m_pid = new PIDController(1, 0, 0, new PIDSource() {
      PIDSourceType m_sourceType = PIDSourceType.kDisplacement;

      @Override
      public double pidGet() {
        return Robot.m_drivetrain.getDistance();
      }

      @Override
      public void setPIDSourceType(PIDSourceType pidSource) {
        m_sourceType = pidSource;
      }

      @Override
      public PIDSourceType getPIDSourceType() {
        return m_sourceType;
      }
    }, d -> Robot.m_drivetrain.driveWithRoation(Robot.m_oi.getJoystick(), d * -1.0));

    m_pid.setAbsoluteTolerance(0.01);
    m_pid.setSetpoint(Robot.m_drivetrain.getHeading());
    m_distance = distance;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    // Get everything in a safe starting state.
    Robot.m_drivetrain.reset();
    m_timer.reset();
    m_timer.start();
    m_pid.reset();
    m_pid.enable();
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    //return m_pid.onTarget();
    return m_timer.get() > m_distance * 1000;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    // Stop PID and the wheels
    m_pid.disable();
    Robot.m_drivetrain.drive(0, 0);
  }
}
