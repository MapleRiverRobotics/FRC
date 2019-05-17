/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc7541.RocketBot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class DriveToLineCommandGroup extends CommandGroup {

    private static final double distanceFromSensorToCenter = -10;

    public DriveToLineCommandGroup() {
        //addSequential(new CrossLineAndStopCommand(-0.5));
        addSequential(new StopOnLineCommand(0.6));
        addSequential(new DriveStraightCommand(distanceFromSensorToCenter));
    }
}