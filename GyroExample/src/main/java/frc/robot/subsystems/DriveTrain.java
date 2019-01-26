package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Robot;
import frc.robot.UltrasonicMB1013;
import frc.robot.commands.ArcadeDriveWithJoystick;

/**
 * The DriveTrain subsystem incorporates the sensors and actuators attached to
 * the robots chassis. These include four drive motors, a left and right encoder
 * and a gyro.
 */
public class DriveTrain extends Subsystem {
  WPI_TalonSRX m_leftMaster = new WPI_TalonSRX(1);
  WPI_TalonSRX m_leftSlave = new WPI_TalonSRX(2);
  WPI_TalonSRX m_rightSlave = new WPI_TalonSRX(3);
  WPI_TalonSRX m_rightMaster = new WPI_TalonSRX(4);

  // private final SpeedController m_leftMotor
  //     = new SpeedControllerGroup(new WPI_TalonSRX(1), new WPI_TalonSRX(2));
  // private final SpeedController m_rightMotor
  //     = new SpeedControllerGroup(new WPI_TalonSRX(3), talonWithGyro);

  private final DifferentialDrive m_drive = new DifferentialDrive(m_leftMaster, m_rightMaster);

  private final Encoder m_leftEncoder = new Encoder(1, 2);
  private final Encoder m_rightEncoder = new Encoder(3, 4);
  private final UltrasonicMB1013 m_rangefinder = new UltrasonicMB1013(0);
  private final PigeonIMU m_gyro = new PigeonIMU(m_rightSlave);

  /**
   * Create a new drive train subsystem.
   */
  public DriveTrain() {
    super();

    m_leftMaster.setInverted(true);
    m_leftSlave.follow(m_leftMaster);
    m_leftSlave.setInverted(true);

    m_rightMaster.setInverted(true);
    m_rightSlave.follow(m_rightMaster);
    m_rightSlave.setInverted(true);

    // Encoders may measure differently in the real world and in
    // simulation. In this example the robot moves 0.042 barleycorns
    // per tick in the real world, but the simulated encoders
    // simulate 360 tick encoders. This if statement allows for the
    // real robot to handle this difference in devices.
    if (Robot.isReal()) {
      m_leftEncoder.setDistancePerPulse(0.042);
      m_rightEncoder.setDistancePerPulse(0.042);
    } else {
      // Circumference in ft = 6in/12(in/ft)*PI
      m_leftEncoder.setDistancePerPulse((6.0 / 12.0 * Math.PI) / 360.0);
      m_rightEncoder.setDistancePerPulse((6.0 / 12.0 * Math.PI) / 360.0);
    }

    // Let's name the sensors on the LiveWindow
    addChild("Drive", m_drive);
    addChild("Left Encoder", m_leftEncoder);
    addChild("Right Encoder", m_rightEncoder);
    addChild("Rangefinder", m_rangefinder);
    addChild(m_gyro);
    m_leftEncoder.setName("_Left Encoder");
    m_rightEncoder.setName("_Right Encoder");
  }

  /**
   * When no other command is running let the operator drive around using the
   * PS3 joystick.
   */
  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new ArcadeDriveWithJoystick());
  }

  /**
   * The log method puts interesting information to the SmartDashboard.
   */
  public void log() {
    SmartDashboard.putNumber("Left Distance", m_leftEncoder.getDistance());
    SmartDashboard.putNumber("Right Distance", m_rightEncoder.getDistance());
    SmartDashboard.putNumber("Left Speed", m_leftEncoder.getRate());
    SmartDashboard.putNumber("Right Speed", m_rightEncoder.getRate());
    SmartDashboard.putNumber("Range", getDistanceToObstacle());
    SmartDashboard.putNumber("Gyro Yaw", getHeading());
  }

  /**
   * Tank style driving for the DriveTrain.
   *
   * @param speed Speed in range [-1,1]
   * @param rotation Rotation in range [-1,1]
   */
  public void drive(double speed, double rotation) {
    SmartDashboard.putNumber("Speed", speed);
    SmartDashboard.putNumber("Rotation", rotation);

    // if joystick not more than 10% throttle/movement, then reset to zero
    if (isSpeedInDeadband(speed)) {
      speed = 0.0;
    }
    if (isRotationInDeadband(rotation)) {
      rotation = 0.0;
    }

    m_drive.arcadeDrive(-speed, rotation);
  }

  public void tankDrive(double leftSpeed, double rightSpeed) {
    SmartDashboard.putNumber("Left Speed", leftSpeed);
    SmartDashboard.putNumber("Right Speed", rightSpeed);

    // // if joystick not more than 10% throttle/movement, then reset to zero
    // if (isSpeedInDeadband(leftSpeed)) {
    //   leftSpeed = 0.0;
    // }
    // if (isSpeedInDeadband(rightSpeed)) {
    //   rightSpeed = 0.0;
    // }

    m_drive.tankDrive(leftSpeed, rightSpeed);
  }

  public void driveWithRoation(double speed, double rotation) {

    // Call the drive method to move the robot
    m_drive.curvatureDrive(speed, rotation, false);
  }

  public void driveWithRoation(Joystick joy, double rotation) {
    double throttleSpeed = calculateSpeed(joy);
    double forwardSpeed = joy.getY() * throttleSpeed;

    // If speed is too low, don't bother turning.  Need movement to turn well.
    if (isSpeedInDeadband(forwardSpeed)) {
      rotation = 0.0;
    }

    // Call the drive method to move the robot
    m_drive.curvatureDrive(forwardSpeed, rotation, false);
  }

  /**
   * Tank style driving for the DriveTrain.
   *
   * @param joy The joystick to use to drive tank style.
   */
  public void drive(Joystick joy) {
    double throttleSpeed = calculateSpeed(joy);
    double forwardSpeed = joy.getY() * throttleSpeed;
    double rotation = joy.getZ() * throttleSpeed;

    // Call the drive method to move the robot
    drive(forwardSpeed, rotation);
  }

  // Calculate the speed based on the speed controller of the joystick
  // The speed controller full forward is -1, full reverse is 1.
  private double calculateSpeed(Joystick joy) {
    return ((joy.getThrottle() * -1.0 + 1.0) / 2.0) * -1.0;
  }

  private boolean isSpeedInDeadband(double speed) {
    return (Math.abs(speed) < 0.1);
  }

  private boolean isRotationInDeadband(double rotation) {
    return (Math.abs(rotation) < 0.1);
  }

  /**
   * Tank style driving for the DriveTrain.
   *
   * @param joy The ps3 style joystick to use to drive tank style.
   */
  public void turnLeft(double rotation) {
    m_drive.arcadeDrive(0, rotation * .4);
  }

  /**
   * Get the robot's heading.
   *
   * @return The robots heading in degrees.
   */
  public double getHeading() {
    double[] ypr = new double[3];
    m_gyro.getYawPitchRoll(ypr);
    return ypr[0];
  }

  /**
   * Reset the robots sensors to the zero states.
   */
  public void reset() {
    m_leftEncoder.reset();
    m_rightEncoder.reset();
  }
  public void resetHeading() {
    m_gyro.setYaw(0);
  }

  /**
   * Get the average distance of the encoders since the last reset.
   *
   * @return The distance driven (average of left and right encoders).
   */
  public double getDistance() {
    return (m_leftEncoder.getDistance() + m_rightEncoder.getDistance()) / 2;
  }

  /**
   * Get the distance to the obstacle in inches
   *
   * @return The distance to the obstacle detected by the rangefinder.
   */
  public double getDistanceToObstacle() {
    return m_rangefinder.getDistanceInInches();
  }
}
