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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;

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

    m_pid = new PIDController(.025, 0.0, 0.5, new PIDSource() {
      PIDSourceType m_sourceType = PIDSourceType.kDisplacement;

      @Override
      public double pidGet() {
        double hypotenus = Robot.m_drivetrain.getDistanceToObstacle();
        double heading = Robot.m_drivetrain.getHeading();

        double distanceToWall = Math.cos(Math.toRadians(heading)) * hypotenus;
        distanceToWall = Math.abs(distanceToWall);

        SmartDashboard.putNumber("Heading", heading);
        SmartDashboard.putNumber("Hypotenus", hypotenus);
        SmartDashboard.putNumber("Distance", distanceToWall);

        return distanceToWall;
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
    d -> Robot.m_drivetrain.driveWithRoation(-.3, d));

    //-0.2 .. -1
    //-1 .. -0.2

    m_pid.setOutputRange(-0.3, 0.3);
    m_pid.setAbsoluteTolerance(1);
    m_distance = distance;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    // Get everything in a safe starting state.
    Robot.m_drivetrain.reset();
    m_pid.reset();
    m_pid.setSetpoint(m_distance);
    m_pid.enable();
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return Robot.m_oi.getJoystick().getRawButtonReleased(10);
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    // Stop PID and the wheels
    m_pid.disable();
    Robot.m_drivetrain.drive(0, 0);
  }
}
