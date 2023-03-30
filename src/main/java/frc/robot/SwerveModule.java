// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveModule {
  private static final double kWheelRadius = 2;
  private static final int kEncoderResolution = 4096;

  private static final double kModuleMaxAngularVelocity = Drivetrain.kMaxAngularSpeed;
  private static final double kModuleMaxAngularAcceleration = 2 * Math.PI; // radians per second squared

  private CANSparkMax driveMotor;
  private CANSparkMax turningMotor;

  private RelativeEncoder driveEncoder;
  private RelativeEncoder turningEncoder;

  private DutyCycleEncoder AnalogEncoder;

  private double offsetAngle;

  public boolean openLoopDrive = false;

  // Gains are for example purposes only - must be determined for your own robot!
  // private final PIDController m_drivePIDController = new PIDController(1, 0, 0);

  //PID Controller, trying SparkMaxPID controllers
  private SparkMaxPIDController drivePIDController, rotatePIDController;
  private PIDController turningPIDController;

  // Gains are for example purposes only - must be determined for your own robot!
  // private final ProfiledPIDController m_turningPIDController = new ProfiledPIDController(1, 0, 0, new TrapezoidProfile.Constraints(kModuleMaxAngularVelocity, kModuleMaxAngularAcceleration));

  //PID setup, one for drive and rotate
  public double kP_drive, 
    kI_drive,
    kD_drive,
    kIz_drive,
    kFF_drive,
    kMaxOutput_drive,
    kMinOutput_drive,
    maxRPM_drive,
    maxVel_drive,
    minVel_drive,
    maxAcc_drive,
    allowedErr_drive;

  public double kP_rotate, 
    kI_rotate,
    kD_rotate,
    kIz_rotate,
    kFF_rotate,
    kMaxOutput_rotate,
    kMinOutput_rotate,
    maxRPM_rotate,
    maxVel_rotate,
    minVel_rotate,
    maxAcc_rotate,
    allowedErr_rotate;

  // Gains are for example purposes only - must be determined for your own robot!
  private final SimpleMotorFeedforward m_driveFeedforward = new SimpleMotorFeedforward(1, 3);
  private final SimpleMotorFeedforward m_turnFeedforward = new SimpleMotorFeedforward(1, 0.5);

  //Main construction for swerve 
  public SwerveModule(CANSparkMax driveMotorChannel, CANSparkMax turningMotorChannel, DutyCycleEncoder analogInput) {
    this.AnalogEncoder = analogInput;
    this.driveMotor = driveMotorChannel;
    this.turningMotor = turningMotorChannel;
    this.driveEncoder = driveMotorChannel.getEncoder();
    this.turningEncoder = turningMotorChannel.getEncoder();

    driveEncoder = driveMotorChannel.getEncoder();
    turningEncoder = turningMotorChannel.getEncoder();

    driveMotor.restoreFactoryDefaults();
    turningMotor.restoreFactoryDefaults();
    // Not sure if needed
    // turningMotor.setInverted(true);


    // driveEncoder.setPositionConversionFactor(4*Math.PI/(6.095));
    // driveEncoder.setVelocityConversionFactor(4*Math.PI/(6.095));

    // turningEncoder.setPositionConversionFactor(2*Math.PI/(13.738));
    // turningEncoder.setVelocityConversionFactor(2*Math.PI/(13.738));

    // m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);

    drivePIDController = driveMotor.getPIDController();

    //Set PID values for drive, needs tuning!! Temp values
    kP_drive = 0;
    kI_drive = 0;
    kD_drive = 0;
    kIz_drive = 0;
    kFF_drive = 0;
    kMaxOutput_drive = 0;
    kMinOutput_drive = 0;
    maxRPM_drive = 0;
    maxVel_drive = 0;
    maxAcc_drive = 0;

    //Set PIDs for drive
    drivePIDController.setP(kP_drive);
    drivePIDController.setI(kI_drive);
    drivePIDController.setD(kD_drive);
    drivePIDController.setIZone(kIz_drive);
    drivePIDController.setFF(kFF_drive);
    drivePIDController.setOutputRange(kMinOutput_drive, kMaxOutput_drive);
    // SmartMotion is used for storing different sets of trapezoidal profiles
    // Downside with smartMotion looking though CheifDelphi is the profile is collected for the velocity controller instead of position controller
    int smartMotionSlot = 0;
    drivePIDController.setSmartMotionMaxVelocity(maxVel_drive, smartMotionSlot);
    drivePIDController.setSmartMotionMinOutputVelocity(minVel_drive, smartMotionSlot);
    drivePIDController.setSmartMotionMaxAccel(maxAcc_drive, smartMotionSlot);
    drivePIDController.setSmartMotionAllowedClosedLoopError(allowedErr_drive, smartMotionSlot);

    // Rotate PID variables setup
    kP_rotate = 0;
    kI_rotate = 0;
    kD_rotate = 0;
    kIz_rotate = 0;
    kFF_rotate = 0;
    kMaxOutput_rotate = 0;
    kMinOutput_rotate = 0;
    maxRPM_rotate = 0;

    maxVel_rotate = 0;
    maxAcc_rotate = 0;

    allowedErr_rotate = 0;

    // Rotate PID
    rotatePIDController.setP(kP_rotate);
    rotatePIDController.setI(kI_rotate);
    rotatePIDController.setD(kD_rotate);
    rotatePIDController.setIZone(kIz_rotate);
    rotatePIDController.setFF(kFF_rotate);

    turningPIDController =  new PIDController(1, 0, 0); //temp
    turningPIDController.enableContinuousInput(-Math.PI, Math.PI);

    resetEncoders();
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(
        getDriveEncoderVelocityMetersPerSec(), new Rotation2d(getDriveVelocity()));
  }

  // Get the current velocity
  public double getDriveVelocity() {
    return driveEncoder.getVelocity();
  } 
  // Get the current velocity in meters by converting RPM to meters/second
  public double getDriveEncoderVelocityMetersPerSec() {
    return getDriveVelocity() * Constants.otherVars.kDriveEncoderDistance / 60; 
  }


  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        driveEncoder.getPosition(), new Rotation2d(turningEncoder.getPosition()));
  }

  // Get the current position
  public double getDriveEncoderPositionMeters() {
    return driveEncoder.getPosition() * Constants.otherVars.kDriveEncoderDistance;
  }
  // Get the module angle 
  // Does this by taking in the raw angle (current rotations times 360 minus the offset )
  public double getModuleAngle() {
    double rawAngle = (turningEncoder.getPosition() * 360 - this.offsetAngle) % 360;
    double angle;
    if (rawAngle > 180.0 && rawAngle < 360.0) {
      angle = -180 + rawAngle % 180;
    } else {
      angle = rawAngle;
    }
    return angle;
  }
  // Get the module randians
  public double getModuleAngleRadians() {
    return getModuleAngle() * Math.PI / 180.0;
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState state) {
    if(Math.abs(state.speedMetersPerSecond) < 0.25) {
      stop();
      return;
    }
    state = SwerveModuleState.optimize(state, new Rotation2d(getModuleAngleRadians()));

    double motorRPM = getMotorRPMFromDriveVelocity(state.speedMetersPerSecond);

    if (Math.abs(state.speedMetersPerSecond) > 0.1 * Constants.otherVars.kMaxSpeedMetersPerSecond) {
      if (openLoopDrive) {
      driveMotor.set(state.speedMetersPerSecond);
      } else {
      drivePIDController.setReference(motorRPM, CANSparkMax.ControlType.kSmartVelocity);
      }
    } else {
      drivePIDController.setReference(0, CANSparkMax.ControlType.kSmartVelocity);
    }

    turningMotor.set(turningPIDController.calculate(getModuleAngleRadians(), state.angle.getRadians()));
  }

  //Motor RPMS 
  public double getMotorRPMFromDriveVelocity(double velocity) {
    return velocity * 60 / Constants.otherVars.kDriveEncoderDistance;
  }
  //Reset the encoders
  public void resetEncoders() {
    driveEncoder.setPosition(0);
    AnalogEncoder.reset();
  }

  //Emergency 
  private void stop() {
    driveMotor.set(0);
    turningMotor.set(0);
  }
}
