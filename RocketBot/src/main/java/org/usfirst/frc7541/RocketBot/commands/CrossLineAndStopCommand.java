/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc7541.RocketBot.commands;

import org.usfirst.frc7541.RocketBot.Robot;
import org.usfirst.frc7541.RocketBot.sensors.LineSensor;
import org.usfirst.frc7541.RocketBot.sensors.LineSensor.LineSensorStatus;

import edu.wpi.first.wpilibj.command.TimedCommand;

public class CrossLineAndStopCommand extends TimedCommand {

    LineSensor lineSensor;
    boolean lineFound;
    double speed;

    public CrossLineAndStopCommand(double speed) {
        super(5);
        requires(Robot.driveTrain);
        this.speed = speed;
    }

    // Called just before this Command runs the first time
    @Override
    protected void initialize() {
        lineSensor = LineSensor.getInstance();
        lineFound = false;
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute() {
        Robot.driveTrain.arcadeDrive(speed, 0);
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    protected boolean isFinished() {
        LineSensorStatus lineStatus = lineSensor.eitherSideLineStatus(10);
        lineFound = lineFound || lineStatus == LineSensorStatus.Centered;
        System.out.println(lineStatus);
        return super.isFinished() || lineStatus == LineSensorStatus.NoReading
                || (lineFound && lineStatus == LineSensorStatus.NotOnLine);
    }

    // Called once after isFinished returns true
    @Override
    protected void end() {
        Robot.driveTrain.stop();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    @Override
    protected void interrupted() {
    }
}
