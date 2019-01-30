/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc7541.RocketBot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class StopOnLineCommandGroup extends CommandGroup {

    public StopOnLineCommandGroup() {
        addSequential(new CrossLineAndStopCommand(-0.5));
        addSequential(new StopOnLineCommand(-0.25));
    }
}
