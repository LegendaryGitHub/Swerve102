// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
// import edu.wpi.first.wpilibj.AnalogInput;
// import edu.wpi.first.wpilibj.AnalogTrigger;
// import edu.wpi.first.wpilibj.DigitalInput;
// import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

public class SwerveModule implements Sendable {
  private static final double kDriveGearing=1/7.13;
  private static final double kWheelRadius = Units.inchesToMeters(2);
  private static final double kAzimuthGearing=1/13.71;

  private static final int kEncoderResolution = 4096;

  private static final double kModuleMaxAngularVelocity = Drivetrain.kMaxAngularSpeed;
  private static final double kModuleMaxAngularAcceleration = 2 * Math.PI; // radians per second squared

  private CANSparkMax driveMotor;
  private CANSparkMax azimuthMotor;

  private RelativeEncoder driveEncoder;
  private RelativeEncoder azimuthEncoder;

  private DutyCycleEncoder absoluteEncoder;

  private double offsetAngle = 0;

  public boolean openLoopDrive = false;

  // Gains are for example purposes only - must be determined for your own robot!
  // private final PIDController m_drivePIDController = new PIDController(1, 0, 0);

  //PID Controller, trying SparkMaxPID controllers
  private SparkMaxPIDController drivePIDController;//, rotatePIDController;
  private final PIDController turningPIDController;
  // private final SparkMaxPIDController drivePIDController;

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
  private final SimpleMotorFeedforward m_driveFeedforward = new SimpleMotorFeedforward(0, 0);
  private final SimpleMotorFeedforward m_turnFeedforward = new SimpleMotorFeedforward(0, 0);

  //Main construction for swerve 
  public SwerveModule(CANSparkMax driveMotorChannel, CANSparkMax turningMotorChannel, DutyCycleEncoder analogInput, Boolean driveInvert) {
    this.absoluteEncoder = analogInput;
    this.driveMotor = driveMotorChannel;
    this.azimuthMotor = turningMotorChannel;
    this.driveEncoder = driveMotorChannel.getEncoder();
    this.azimuthEncoder = turningMotorChannel.getEncoder();
    // this.rotatePIDController = rotatePIDController;

    driveEncoder = driveMotorChannel.getEncoder();
    azimuthEncoder = turningMotorChannel.getEncoder();

    driveMotor.restoreFactoryDefaults();
    azimuthMotor.restoreFactoryDefaults();

    driveMotor.setInverted(false);
    azimuthMotor.setInverted(true);

    drivePIDController = driveMotor.getPIDController();
    // rotatePIDController = azimuthMotor.getPIDController();

    //Set PID values for drive, needs tuning!! Temp values
    kP_drive = 0.001;//1/3
    kI_drive = 0;
    kD_drive = 0;
    kIz_drive = 0;
    kFF_drive = 0;
    kMaxOutput_drive = 1;
    kMinOutput_drive = -1;
    maxRPM_drive = 5700;
    maxVel_drive = 5700;
    maxAcc_drive = 3000;

    // var gearing=1;
    // var kWheelRadius=3;
    driveEncoder.setPositionConversionFactor( 1/kDriveGearing*kWheelRadius*2*Math.PI );
    driveEncoder.setVelocityConversionFactor( 1/kDriveGearing*kWheelRadius*2*Math.PI * 60);

    azimuthEncoder.setPositionConversionFactor(kAzimuthGearing *2*Math.PI);
    azimuthEncoder.setVelocityConversionFactor(kAzimuthGearing *2*Math.PI * 60);

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
    kP_rotate = 0.01;//1/(Math.PI*4);
    kI_rotate = 0.002;
    kD_rotate = 0;
    kIz_rotate = 0;
    kFF_rotate = 0;
    kMaxOutput_rotate = 1;
    kMinOutput_rotate = -1;
    maxRPM_rotate = 5700;

    maxVel_rotate = 5700;
    maxAcc_rotate = 3000;

    allowedErr_rotate = .1;

    // Rotate PID
    // rotatePIDController.setP(kP_rotate);
    // rotatePIDController.setI(kI_rotate);
    // rotatePIDController.setD(kD_rotate);
    // rotatePIDController.setIZone(kIz_rotate);
    // rotatePIDController.setFF(kFF_rotate);

    // turningPIDController =  new PIDController(3*4/(Math.PI*2), 0, 0); //temp
    turningPIDController =  new PIDController(kP_rotate, kI_rotate, kD_rotate);
    turningPIDController.enableContinuousInput(-Math.PI, Math.PI);

    resetEncoders();
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  // public SwerveModuleState getState() {
  //   return new SwerveModuleState(
  //       getDriveEncoderVelocityMetersPerSec(), new Rotation2d(getDriveVelocity()));
  // }

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
        driveEncoder.getPosition(), new Rotation2d(azimuthEncoder.getPosition()));
  }

  // Get the current position
  public double getDriveEncoderPositionMeters() { //Note, seems to be in inches not meters
    return driveEncoder.getPosition() * Constants.otherVars.kDriveEncoderDistance;
  }
  // Get the module angle 
  // Does this by taking in the raw angle (current rotations times 360 minus the offset )
  public double getAbsoluteAngle() {
    double rawAngle = absoluteEncoder.getAbsolutePosition();//*Math.PI*2; //.getDistance
    return MathUtil.angleModulus(rawAngle);
    // return rawAngle;
  }
  // Get the module randians
  public double getAngleRadians() {
    return MathUtil.angleModulus( azimuthEncoder.getPosition() );
  }
  public double getAngleDegrees() {
    return Math.toDegrees( azimuthEncoder.getPosition() );
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
    // state = SwerveModuleState.optimize(state, new Rotation2d(getAngleRadians()));

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

    azimuthMotor.set(turningPIDController.calculate(getAngleRadians(), state.angle.getRadians()));
  }

  //Motor RPMS 
  public double getMotorRPMFromDriveVelocity(double velocity) {
    return velocity * 60 / Constants.otherVars.kDriveEncoderDistance;
  }
  //Reset the encoders
  public void resetEncoders() {
    driveEncoder.setPosition(0);
    azimuthEncoder.setPosition(0);
  }

  //Emergency 
  private void stop() {
    driveMotor.set(0);
    azimuthMotor.set(0);
  }
  
  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("SwerveModule");
    builder.addDoubleProperty("angle(abs)", this::getAbsoluteAngle, null);
    builder.addDoubleProperty("distance", ()->driveEncoder.getPosition(), null);
    builder.addDoubleProperty("angle(rad)", this::getAngleRadians, null);
    builder.addDoubleProperty("angle(deg)", this::getAngleDegrees, null);
  }
}
