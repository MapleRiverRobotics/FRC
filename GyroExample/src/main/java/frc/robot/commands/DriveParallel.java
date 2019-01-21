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
import frc.robot.UltrasonicMB1013;

/**
 * Drive the parallel next to a wall.  This uses the ultrasonic sensor to determine
 * how far from the wall the robot is, and then drives it towards the wall while
 * moving mostly forward.
 */
public class DriveParallel extends Command {
  private final PIDController m_pid;

  private double m_distance;
 
  /**
   * Create a new DriveStraight command.
   * @param distance The distance to drive
   */
  public DriveParallel(double distance) {
    requires(Robot.m_drivetrain);

    m_pid = new PIDController(1, 0, 0, new PIDSource() {
      PIDSourceType m_sourceType = PIDSourceType.kDisplacement;

      @Override
      public double pidGet() {
        return Robot.m_drivetrain.getDistanceToObstacle();
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
    d -> Robot.m_drivetrain.driveWithRoation(Robot.m_oi.getJoystick(), d));

    m_pid.setAbsoluteTolerance(10);
    m_distance = distance;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    // Get everything in a safe starting state.
    Robot.m_drivetrain.reset();
    m_pid.reset();
    m_pid.setSetpoint(Robot.m_drivetrain.getDistanceToObstacle() - m_distance);
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
