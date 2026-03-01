// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.*;
import frc.robot.commands.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.io.File;
import java.util.List;

import javax.imageio.plugins.tiff.GeoTIFFTagSet;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private SwerveSubsystem m_swerveDrive;
  private Shooter m_Shooter =  new Shooter();
  private Climber m_climber = new Climber(); 
  private Hopper m_hopper   = new Hopper();
  private Intake m_intake   = new Intake();
  private Lights m_Lights = new Lights();

  private CommandXboxController joystick1, xboxController;
  final SendableChooser<Command> autoChooser;
  final SendableChooser<Boolean> driveChooser= new SendableChooser<>();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    m_swerveDrive =  new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),"swerve"));
    NamedCommands.registerCommand("TurnToTarget", new AimAtHub(m_swerveDrive, joystick1, m_Lights));
    NamedCommands.registerCommand("ToggleShoot", new Shoot(m_Shooter, m_intake));
    NamedCommands.registerCommand("ToggleIntake", m_intake.toggleIntake());
    NamedCommands.registerCommand("ToggleHopper", m_hopper.toggleHopperCommand());
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
      List<PathPlannerPath> pathsInAuto = null;
      try{
        pathsInAuto = PathPlannerAuto.getPathGroupFromAutoFile(selectedOption.getName());
      }catch( Exception ex){ 
        System.out.println("Failed to parse autopath"+selectedOption.getName());

      }
      Pose2d startingPose = new Pose2d(0,0,Rotation2d.kZero);
      if (!pathsInAuto.isEmpty()) {
        PathPlannerPath path0 = pathsInAuto.get(0);
        startingPose = new Pose2d(path0.getPoint(0).position, path0.getIdealStartingState().rotation());
      }
      m_swerveDrive.resetOdometry(startingPose);
    
      System.out.println("Selected option: " + selectedOption + "   "+selectedOption.getName());
      System.out.println("pose: "+startingPose.getX()+", "+startingPose.getY());
      
      // Perform actions based on the selected option
      });

    try {
      joystick1 = new CommandXboxController(Constants.controller.xboxPort1);
      xboxController = new CommandXboxController(Constants.controller.xboxPort2); // Creates an XboxController on port 2.
    } catch (RuntimeException ex) {
      DriverStation.reportError("Error instantiating Xboxcontroller: " + ex.getMessage(), true);
    }
       
    Command oneDrive = m_swerveDrive.oneDriveCommand(
      () -> MathUtil.applyDeadband(-joystick1.getRawAxis(1), Constants.controller.LEFT_Y_DEADBAND), // X
      () -> MathUtil.applyDeadband(-joystick1.getRawAxis(0), Constants.controller.LEFT_X_DEADBAND), // Y 
      () -> -joystick1.getRawAxis(4),                                                               // Angle 1
      () -> -joystick1.getRawAxis(5)                                                              // Angle 2
      );  
    
      /* -> driveCommand can take a single value for rotation or two for the specific target angle -> so take angle and pass cos(theta), sin(theta)
        Might also need to turn on heading correct
        Call just one drive command here, split options in servedrive.
      */
    m_swerveDrive.setDefaultCommand(oneDrive);

    SmartDashboard.putData("Auto Chooser", autoChooser);
    SmartDashboard.putData("Turn To Hub", new AimAtHub(m_swerveDrive, joystick1, m_Lights));
    SmartDashboard.putData("drive",new AutoWaypoints(m_swerveDrive,  new Pose2d(3.235,7.186,Rotation2d.fromDegrees(-78.024))));
    SmartDashboard.putData("Blue Lights",m_Lights.blue());
    SmartDashboard.putData("Lights off",m_Lights.off());
    SmartDashboard.putData("Red Lights",m_Lights.red());
    SmartDashboard.putData("Rainbow Lights",m_Lights.rainbow());
    SmartDashboard.putData("Left",new AutoWaypoints(m_swerveDrive, new Pose2d(3.235,7.186,Rotation2d.fromDegrees(-78.024))));
    SmartDashboard.putData("Up", new AutoWaypoints(m_swerveDrive, new Pose2d(2.847,4.019,Rotation2d.fromDegrees(0))));
    SmartDashboard.putData("Down",new AutoWaypoints(m_swerveDrive, new Pose2d(1.804,3.965,Rotation2d.fromDegrees(0))));
    SmartDashboard.putData("Right", new AutoWaypoints(m_swerveDrive, new Pose2d(2.901,0.963,Rotation2d.fromDegrees(47.545))));
    //Warm up Path following commands
    FollowPathCommand.warmupCommand();
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
    /* Driver Controls Port 1 */
    joystick1.b().onTrue(new AimAtHub(m_swerveDrive, joystick1, m_Lights));
    joystick1.back().onTrue( m_swerveDrive.ToggleBrake());
    joystick1.y().onTrue(new Shoot(m_Shooter, m_intake));
    joystick1.povLeft().onTrue(new AutoWaypoints(m_swerveDrive, new Pose2d(3.235,7.186,Rotation2d.fromDegrees(-78.024))));
    joystick1.povUp().onTrue(new AutoWaypoints(m_swerveDrive, new Pose2d(2.847,4.019,Rotation2d.fromDegrees(0))));
    joystick1.povDown().onTrue(new AutoWaypoints(m_swerveDrive, new Pose2d(1.804,3.965,Rotation2d.fromDegrees(0))));
    joystick1.povRight().onTrue(new AutoWaypoints(m_swerveDrive, new Pose2d(2.901,0.963,Rotation2d.fromDegrees(47.545))));
    
    /*Co-driver controls  Port 2 */
    xboxController.povUp().onTrue( m_climber.Up());
    xboxController.povDown().onTrue(m_climber.Down());
    xboxController.rightTrigger().onTrue(new SequentialCommandGroup(m_Lights.red(),m_intake.toggleIntake()));
    xboxController.a().onTrue(m_hopper.toggleHopperCommand());
   
    }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */

  public Command getAutonomousCommand() {

    Command selected = autoChooser.getSelected();
    //String autoName = autoChooser.getSelected().getName();
    //PathPlannerAuto AutoPath = new PathPlannerAuto(autoName);
    //m_swerveDrive.resetOdometry(AutoPath.getStartingPose());
    return selected;
    
  }
}