// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Represents a swerve drive style drivetrain. */
public class Drivetrain {
  public static final double kMaxSpeed = 3.0; // 3 meters per second
  public static final double kMaxAngularSpeed = Math.PI; // 1/2 rotation per second

  private final Translation2d m_frontLeftLocation = new Translation2d(0.381, 0.381);
  private final Translation2d m_frontRightLocation = new Translation2d(0.381, -0.381);
  private final Translation2d m_backLeftLocation = new Translation2d(-0.381, 0.381);
  private final Translation2d m_backRightLocation = new Translation2d(-0.381, -0.381);

  private final SwerveModule m_frontLeft = new SwerveModule(
      new CANSparkMax(Constants.DrivetainMotors.kDriveMotorFrontLeft, MotorType.kBrushless), 
      new CANSparkMax(Constants.DrivetainMotors.kRotationMotorFrontLeft, MotorType.kBrushless), 
      new DutyCycleEncoder(Constants.AnalogEncoders.encoderFrontLeft),
      false
      ); //1, 2, 0, 1, 2, 3
  private final SwerveModule m_frontRight = new SwerveModule(
    new CANSparkMax(Constants.DrivetainMotors.kDriveMotorFrontRight, MotorType.kBrushless), 
    new CANSparkMax(Constants.DrivetainMotors.kRotationMotorFrontRight, MotorType.kBrushless), 
    new DutyCycleEncoder(Constants.AnalogEncoders.encoderFrontRight),
    false
    );
  private final SwerveModule m_backLeft = new SwerveModule(
    new CANSparkMax(Constants.DrivetainMotors.kDriveMotorBackLeft, MotorType.kBrushless),
    new CANSparkMax(Constants.DrivetainMotors.kRotationMotorBackLeft, MotorType.kBrushless),
    new DutyCycleEncoder(Constants.AnalogEncoders.encoderBackLeft), true
  );
  private final SwerveModule m_backRight = new SwerveModule(
    new CANSparkMax(Constants.DrivetainMotors.kDriveMotorBackRight, MotorType.kBrushless),
    new CANSparkMax(Constants.DrivetainMotors.kRotationMotorBackRight, MotorType.kBrushless),
    new DutyCycleEncoder(Constants.AnalogEncoders.encoderBackRight), 
    true
  );


  public AHRS navx = new AHRS(Port.kMXP);
  
  private final SwerveDriveKinematics m_kinematics =
      new SwerveDriveKinematics(
          m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

  private final SwerveDriveOdometry m_odometry =
      new SwerveDriveOdometry(
          m_kinematics,
          navx.getRotation2d(),
          new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_backLeft.getPosition(),
            m_backRight.getPosition()
          });

  public Drivetrain() {
    navx.reset();

    //Rotation/Drive/AbsEncoder SmartDashboard
    SmartDashboard.putData("drivetrain/backRight/",m_backRight);
    SmartDashboard.putData("drivetrain/backLeft/",m_backLeft);
    SmartDashboard.putData("drivetrain/frontRight/",m_frontRight);
    SmartDashboard.putData("drivetrain/frontLeft/",m_frontLeft);
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed Speed of the robot in the x direction (forward).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    var  swerveModuleStates =
        m_kinematics.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, navx.getRotation2d())
                : new ChassisSpeeds(xSpeed, ySpeed, rot));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, kMaxSpeed);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_backLeft.setDesiredState(swerveModuleStates[2]);
    m_backRight.setDesiredState(swerveModuleStates[3]);
  }

  /** Updates the field relative position of the robot. */
  public void updateOdometry() {
    m_odometry.update(
        navx.getRotation2d(),
        new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_backLeft.getPosition(),
          m_backRight.getPosition()
        });
  }
}
