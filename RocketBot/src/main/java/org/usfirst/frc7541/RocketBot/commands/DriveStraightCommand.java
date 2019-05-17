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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DriveStraightCommand extends Command {
    private double m_distance;
    private PIDController m_pid;

    public DriveStraightCommand(double distance) {
        super(5.0); // set the timeout period as a safety catch in case it fails to stop normally
        m_distance = distance;
        requires(Robot.driveTrain);
    }

    // Called just before this Command runs the first time
    @Override
    protected void initialize() { 
        Robot.driveTrain.resetPosition();

        double kP = SmartDashboard.getNumber("P_Drive", 0.05);
        double kI = SmartDashboard.getNumber("I_Drive", 0.0);
        double kD = SmartDashboard.getNumber("D_Drive", 0.0);

        SmartDashboard.putNumber("P_Drive", kP);
        SmartDashboard.putNumber("I_Drive", kI);
        SmartDashboard.putNumber("D_Drive", kD);

        System.out.print("P: ");
        System.out.println(kP);

        m_pid = new PIDController(kP, kI, kD, new PIDSource() {
            PIDSourceType m_sourceType = PIDSourceType.kDisplacement;

            @Override
            public double pidGet() {
                return Robot.driveTrain.getPosition();
            }

            @Override
            public void setPIDSourceType(PIDSourceType pidSource) {
                m_sourceType = pidSource;
            }

            @Override
            public PIDSourceType getPIDSourceType() {
                return m_sourceType;
            }
        }, d ->  {
            System.out.print("d: ");
            System.out.print(d);
            d = minimumSpeed(d);
            System.out.print(" ");
            System.out.println(d);
            // if (distance > 0) {
                Robot.driveTrain.arcadeDrive(-d, 0);
            // }
            // else {
            //     Robot.driveTrain.arcadeDrive(0.5, 0);
            // }
        });

        //m_pid.setInputRange(-500, 500);
        m_pid.setOutputRange(-.6, .6);
        m_pid.setAbsoluteTolerance(.4);

        m_pid.reset();
        m_pid.setSetpoint(m_distance);
        m_pid.enable();
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute() {

    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    protected boolean isFinished() {
        boolean timedOut = isTimedOut();
        if (timedOut) {
            System.out.println("Drive straight timed out");
        }
        return m_pid.onTarget() || isTimedOut();
    }

    // Called once after isFinished returns true
    @Override
    protected void end() {
        Robot.driveTrain.stop();
        m_pid.disable();
    }

    private double minimumSpeed(double d) {
        double minSpeed = .3;
        if (d> 0 && d < minSpeed) {
            d = minSpeed;
        } else if (d < 0 && d > -minSpeed) {
            d = -minSpeed;
        }
        return d;
    }

}
