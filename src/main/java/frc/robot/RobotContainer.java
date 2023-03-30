// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class RobotContainer extends CommandBase {
  private Drivetrain drivetrain = new Drivetrain();
  private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(0.01);
  private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(0.01);
  private final SlewRateLimiter m_rotLimiter =  new SlewRateLimiter(0.01);
  double tau = 2*Math.PI;
  double autoWaitTimer = 0;

  public Joystick driver = new Joystick(0);
  SendableChooser<Command> autoChooser = new SendableChooser<>();
  
  /**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including 
 * subsystems, commands, and button mappings) should be declared here.
 */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
  }
  // The robot's subsystems and commands are defined here...
 
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public void periodic() {
    driveWithJoystick(false);
    drivetrain.updateOdometry();
  }
  public void driveWithJoystick(boolean fieldRelative) {
    double xSpeed = -m_xspeedLimiter.calculate(MathUtil.applyDeadband(driver.getRawAxis(3), 0.02));
    xSpeed = driver.getRawAxis(3);

    double ySpeed = -m_yspeedLimiter.calculate(MathUtil.applyDeadband(driver.getRawAxis(2), 0.02));
    ySpeed = -driver.getRawAxis(2);

    double rot = -m_rotLimiter.calculate(MathUtil.applyDeadband(driver.getRawAxis(0), 0.02));
    rot = driver.getRawAxis(0);

    drivetrain.drive(xSpeed, ySpeed, rot, fieldRelative);
  }
  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

  }
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomCommand() {
    // An ExampleCommand will run in autonomous
    return autoChooser.getSelected();
  }
}
