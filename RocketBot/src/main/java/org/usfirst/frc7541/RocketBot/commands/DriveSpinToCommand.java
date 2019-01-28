/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc7541.RocketBot.commands;

import org.usfirst.frc7541.RocketBot.Robot;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.TimedCommand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Add your docs here.
 */
public class DriveSpinToCommand extends Command {

    private PIDController m_pid;
    private double m_degrees;

    /**
     * Add your docs here.
     */
    public DriveSpinToCommand(double angle) {
        super(7.0); // set the timeout period as a safety catch in case turning fails
        // Use requires() here to declare subsystem dependencies
        requires(Robot.driveTrain);

        m_degrees = angle;
    }

    // Called just before this Command runs the first time
    @Override
    protected void initialize() {

        double kP = SmartDashboard.getNumber("P_Spin", 0.05);
        double kI = SmartDashboard.getNumber("I_Spin", 0.0025);
        double kD = SmartDashboard.getNumber("D_Spin", 0.15);

        SmartDashboard.putNumber("P_Spin", kP);
        SmartDashboard.putNumber("I_Spin", kI);
        SmartDashboard.putNumber("D_Spin", kD);

        System.out.print("P: ");
        System.out.println(kP);

        m_pid = new PIDController(kP, kI, kD, new PIDSource() {
            PIDSourceType m_sourceType = PIDSourceType.kDisplacement;

            @Override
            public double pidGet() {
                return Robot.driveTrain.getHeading();
            }

            @Override
            public void setPIDSourceType(PIDSourceType pidSource) {
                m_sourceType = pidSource;
            }

            @Override
            public PIDSourceType getPIDSourceType() {
                return m_sourceType;
            }
        }, d -> Robot.driveTrain.tankDrive(d, -d));

        m_pid.setInputRange(0, 360);
        m_pid.setOutputRange(-.3, .3);
        m_pid.setContinuous();
        m_pid.setAbsoluteTolerance(.5);

        m_pid.reset();
        m_pid.setSetpoint(m_degrees);
        m_pid.enable();
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute() {
    }

    @Override
    protected void end() {
        // Stop PID and the wheels
        Robot.driveTrain.stop();
        m_pid.disable();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    @Override
    protected void interrupted() {
    }

    @Override
    protected boolean isFinished() {
        return m_pid.onTarget();// || isTimedOut();
    }
}
