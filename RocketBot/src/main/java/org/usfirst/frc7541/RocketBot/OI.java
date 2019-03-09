// RobotBuilder Version: 2.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.

package org.usfirst.frc7541.RocketBot;

import org.usfirst.frc7541.RocketBot.commands.*;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.*;
import org.usfirst.frc7541.RocketBot.subsystems.*;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
    //// CREATING BUTTONS
    // One type of button is a joystick button which is any button on a joystick.
    // You create one by telling it which joystick it's on and which button
    // number it is.
    // Joystick stick = new Joystick(port);
    // Button button = new JoystickButton(stick, buttonNumber);

    // There are a few additional built in buttons you can use. Additionally,
    // by subclassing Button you can create custom triggers and bind those to
    // commands the same as any other Button.

    //// TRIGGERING COMMANDS WITH BUTTONS
    // Once you have a button, it's trivial to bind it to a button in one of
    // three ways:

    // Start the command when the button is pressed and let it run the command
    // until it is finished as determined by it's isFinished method.
    // button.whenPressed(new ExampleCommand());

    // Run the command while the button is being held down and interrupt it once
    // the button is released.
    // button.whileHeld(new ExampleCommand());

    // Start the command when the button is released and let it run the command
    // until it is finished as determined by it's isFinished method.
    // button.whenReleased(new ExampleCommand());

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
    public Joystick joystickDrive = new Joystick(0);
    public JoystickButton driveStraightButton;
    public JoystickButton turnToNinetyButton;
    public JoystickButton turnToOneEightyButton;
    public JoystickButton turnToTwoSeventyButton;
    public JoystickButton turnToZeroButton;
    public JoystickButton rotateRightButton;
    public JoystickButton rotateLeftButton;
    public JoystickButton HAB3Reverse;
    public JoystickButton crossLineAndStopButton;

    public Joystick joystickOperator = new Joystick(1);
    public JoystickButton armUpButton;
    public JoystickButton armDownButton;
    public JoystickButton extenderOut;
    public JoystickButton extenderIn;
    public JoystickButton rotatorLeft;
    public JoystickButton rotatorForward;
    public JoystickButton rotatorRight;
    public JoystickButton rotateRight;
    public JoystickButton rotateLeft;
    public JoystickButton liftUpButton;
    public JoystickButton liftDownButton;
    

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS

    public OI() {

        if (joystickDrive != null) {
            liftDownButton = new JoystickButton(joystickDrive, 4);
            liftDownButton.whileHeld(new LiftDown());
            liftUpButton = new JoystickButton(joystickDrive, 6);
            liftUpButton.whileHeld(new LiftUp());

            driveStraightButton = new JoystickButton(joystickDrive, 8);
            driveStraightButton.whenPressed(new DriveStraightCommand(10));
            turnToNinetyButton = new JoystickButton(joystickDrive, 9);
            turnToNinetyButton.whenPressed(new DriveSpinToCommand(90));
            turnToOneEightyButton = new JoystickButton(joystickDrive, 10);
            turnToOneEightyButton.whenPressed(new DriveSpinToCommand(180));
            turnToTwoSeventyButton = new JoystickButton(joystickDrive, 11);
            turnToTwoSeventyButton.whenPressed(new DriveSpinToCommand(270));
            turnToZeroButton = new JoystickButton(joystickDrive, 12);
            turnToZeroButton.whenPressed(new DriveSpinToCommand(0));
            rotateRightButton = new JoystickButton(joystickDrive, 6);
            rotateRightButton.whileHeld(new RotateRight());
            rotateLeftButton = new JoystickButton(joystickDrive, 5);
            rotateLeftButton.whileHeld(new RotateLeft());
            HAB3Reverse = new JoystickButton(joystickDrive, 7);
            HAB3Reverse.whenPressed(new HAB3Reverse());

            crossLineAndStopButton = new JoystickButton(joystickDrive, 3);
            crossLineAndStopButton.whenPressed(new DriveToLineCommandGroup());
        }

        if (joystickOperator != null) {
            rotatorRight = new JoystickButton(joystickOperator, 6);
            rotatorRight.whenPressed(new RotatorRotate(90));
            rotatorForward = new JoystickButton(joystickOperator, 10);
            rotatorForward.whenPressed(new RotatorRotate(0));
            rotatorLeft = new JoystickButton(joystickOperator, 5);
            rotatorLeft.whenPressed(new RotatorRotate(-90));
            armDownButton = new JoystickButton(joystickOperator, 1);
            armDownButton.whileHeld(new ArmDown());
            armUpButton = new JoystickButton(joystickOperator, 4);
            armUpButton.whileHeld(new ArmUp());
            rotateRight = new JoystickButton(joystickOperator, 2);
            rotateRight.whileHeld(new RotateRight());
            rotateLeft = new JoystickButton(joystickOperator, 3);
            rotateLeft.whileHeld(new RotateLeft());
        }

        // SmartDashboard Buttons
        SmartDashboard.putData("Autonomous Command", new AutonomousCommand());
        SmartDashboard.putData("TeleOpDriveCommand", new DriveTeleopCommand());
        SmartDashboard.putData("ExtenderExtendCommand", new ExtenderExtendCommand());
        SmartDashboard.putData("ExtenderRetractCommand", new ExtenderRetractCommand());
        SmartDashboard.putData("RotatorRotateLeft", new RotatorRotate(-90));
        SmartDashboard.putData("RotatorRotateRight", new RotatorRotate(90));
        SmartDashboard.putData("RotatorRotateForward", new RotatorRotate(0));
        SmartDashboard.putData("ArmUp", new ArmUp());
        SmartDashboard.putData("ArmDown", new ArmDown());
    }

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=FUNCTIONS
    public Joystick getJoystickDrive() {
        return joystickDrive;
    }

    public Joystick getJoystickOperator() {
        return joystickOperator;
    }

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=FUNCTIONS

    // driver controls
    public double getJoystickDriveForwardSpeed() {
        if (joystickDrive != null) {
            if (joystickDrive.getRawButton(2)) {
                return joystickDrive.getY() * -1;
            } else {
                return joystickDrive.getY();
            }
        }
        return joystickOperator.getY();
    }

    public double getJoystickOperatorExtenderSpeed() {
        if (joystickOperator != null) {
            int extenderSpeed = joystickOperator.getPOV();
            if(extenderSpeed == 0) {
                return 1;
            }else if(extenderSpeed == 180) {
                return -1;
            }
            return 0;
        }
        return 0;
    }

    public double getJoystickDriveThrottleSpeed() {
        if (joystickDrive != null) {
            return (joystickDrive.getThrottle() * -1.0 + 1.0) / 2.0;
        }
        return .7;
    }

    public double getJoystickDriveRotation() {
        if (joystickDrive != null) {
            return joystickDrive.getZ() * -0.9;
        }
        return joystickOperator.getX() * .9;
    }
}
