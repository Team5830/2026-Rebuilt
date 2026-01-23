// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.Autos;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import java.io.File;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private SwerveSubsystem m_swerveDrive;
  private CommandXboxController joystick1;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    m_swerveDrive =  new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),"swerve"));
    try {
      joystick1 = new CommandXboxController(Constants.controller.xboxPort1);
    } catch (RuntimeException ex) {
      DriverStation.reportError("Error instantiating Xboxcontroller1: " + ex.getMessage(), true);
    }
    Command driveRobotOriented = m_swerveDrive.driveRobotRelative(
      () -> MathUtil.applyDeadband(-joystick1.getRawAxis(1), Constants.controller.LEFT_Y_DEADBAND),
      () -> MathUtil.applyDeadband(-joystick1.getRawAxis(0), Constants.controller.LEFT_X_DEADBAND),
      () -> -joystick1.getRawAxis(4));

     Command driveFieldOrientedDirectAngle = m_swerveDrive.driveCommand(
        () -> MathUtil.applyDeadband(-joystick1.getRawAxis(1), Constants.controller.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(-joystick1.getRawAxis(0), Constants.controller.LEFT_X_DEADBAND),
        () -> MathUtil.applyDeadband(-joystick1.getRawAxis(5), .01));

      Command driveFieldOrientedDirectAngleSim = m_swerveDrive.simDriveCommand(
        () -> MathUtil.applyDeadband(-joystick1.getRawAxis(1), Constants.controller.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(-joystick1.getRawAxis(0), Constants.controller.LEFT_X_DEADBAND),
        () -> -joystick1.getRawAxis(5),
        () -> joystick1.getRawAxis(4)
      );
    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto(m_swerveDrive);
  }
}
