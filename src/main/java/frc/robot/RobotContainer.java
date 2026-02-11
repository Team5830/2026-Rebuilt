// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.*;
import frc.robot.commands.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.io.File;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;


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
  final SendableChooser<Command> autoChooser;
  final SendableChooser<Boolean> driveChooser= new SendableChooser<>();
  Command driveCommand;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    m_swerveDrive =  new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),"swerve"));
    NamedCommands.registerCommand("TurnToTarget", new AimAtHub(m_swerveDrive, joystick1));
    driveChooser.setDefaultOption("FieldOrientedDrive",Boolean.TRUE);
    driveChooser.addOption("RobotOrientedDrive",Boolean.FALSE);
    driveChooser.onChange((selectedOption)->{
      m_swerveDrive.driveField = selectedOption;
      System.out.println("field drive value"+selectedOption);
    });
    // Autochooser must be setup after the named commands
    autoChooser = AutoBuilder.buildAutoChooser("Auto1");
    autoChooser.onChange((selectedOption) -> {
      // This code will be executed whenever the selected option changes
      PathPlannerAuto AutoPath = new PathPlannerAuto(selectedOption);
      //m_swerveDrive.resetOdometry(AutoPath.getStartingPose());
      m_swerveDrive.resetOdometry(new Pose2d(0.7,7.3,Rotation2d.fromDegrees(0)));
      System.out.println("Selected option: " + selectedOption);

      // Perform actions based on the selected option
      });
      driveCommand = new DriveToPointAuto(m_swerveDrive);

    try {
      joystick1 = new CommandXboxController(Constants.controller.xboxPort1);
    } catch (RuntimeException ex) {
      DriverStation.reportError("Error instantiating Xboxcontroller1: " + ex.getMessage(), true);
    }


    
    Command oneDrive = m_swerveDrive.oneDriveCommand(
      () -> MathUtil.applyDeadband(-joystick1.getRawAxis(1), Constants.controller.LEFT_Y_DEADBAND), // X
      () -> MathUtil.applyDeadband(-joystick1.getRawAxis(0), Constants.controller.LEFT_X_DEADBAND), // Y 
      () -> -joystick1.getRawAxis(4),                                                               // Angle 1
      () -> -joystick1.getRawAxis(5)                                                                // Angle 2
      );

      /* -> driveCommand can take a single value for rotation or two for the specific target angle -> so take angle and pass cos(theta), sin(theta)
        Might also need to turn on heading correct
        Call just one drive command here, split options in servedrive.
      */
    m_swerveDrive.setDefaultCommand(oneDrive);
    SmartDashboard.putData("Auto Chooser", autoChooser);
    SmartDashboard.putData("Turn To Hub", new AimAtHub(m_swerveDrive, joystick1));
    SmartDashboard.putData("drive",driveChooser);
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
    joystick1.b().onTrue(new AimAtHub(m_swerveDrive, joystick1));
    joystick1.a().whileTrue(m_swerveDrive.generateCommand( new Pose2d(3.375, 6.364, Rotation2d.fromDegrees(-92.0))) );
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */

  public Command getAutonomousCommand() {

    Command selected = autoChooser.getSelected();
    String autoName = autoChooser.getSelected().getName();
    PathPlannerAuto AutoPath = new PathPlannerAuto(autoName);
    m_swerveDrive.resetOdometry(AutoPath.getStartingPose());
    return selected;
    
  }
}
