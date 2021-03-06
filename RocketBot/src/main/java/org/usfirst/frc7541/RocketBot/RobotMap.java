/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc7541.RocketBot;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {

	/*----------------DRIVE----------------*/
	public static final int driveLeftMaster = 1; 
	public static final int driveLeftSlave = 2; 
	public static final int driveRightSlave = 3; 
    public static final int driveRightMaster = 4; 
    public static final int driveGyro = 3; 
    public static final int driveUltrasonicSensor = 0; 
	/*-------------------------------------*/

	/*---------------ROTATOR---------------*/
    public static final int rotatorMotor = 5;  
    public static final int rotatorDIOChannel1 = 1;  
    public static final int rotatorDIOChannel2 = 2;  
	/*-------------------------------------*/

	/*----------------EXTENDER-----------------*/
    public static final int extenderMotor = 7;  
    public static final int extenderRetractLimitSwitchDio = 6;  
    /*-------------------------------------*/

	/*----------------ARM---------------*/
    public static final int armMotor = 6;  
    public static final int armLowerLimitSwitchDio = 7;  
    public static final int armRaiseLimitSwitchDio = 8;  
    public static final double armSpeed = .4;  // speed which the MiniCIM runs to raise/lower arm
	/*-------------------------------------*/

	/*----------------LIFT---------------*/
    public static final int liftMotor = 8;  
    public static final double liftSpeed = .8;  // speed which the CIM runs to power lift
	/*-------------------------------------*/
}
