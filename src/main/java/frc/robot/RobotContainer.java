// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.*;
import frc.robot.Constants.intake;
import frc.robot.commands.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
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
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import com.pathplanner.lib.util.FlippingUtil;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private SwerveSubsystem m_swerveDrive;
  private NewSwerve m_swerve;
  private Shooter m_Shooter =  new Shooter();
  //private Climber m_climber = new Climber(); 
  private Hopper m_hopper   = new Hopper();
  private Intake m_intake   = new Intake();
  private Lights m_Lights = new Lights();
  Command driveCmd = null;

  private CommandXboxController joystick1, xboxController;
  final SendableChooser<Command> autoChooser;
  final SendableChooser<Boolean> driveChooser= new SendableChooser<>();
  boolean FieldOrientedDrive = false;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    m_swerve =  new NewSwerve(new File(Filesystem.getDeployDirectory(),"swerve"));
    try {
      joystick1 = new CommandXboxController(Constants.controller.xboxPort1);
      xboxController = new CommandXboxController(Constants.controller.xboxPort2); // Creates an XboxController on port 2.
    } catch (RuntimeException ex) {
      DriverStation.reportError("Error instantiating Xboxcontroller: " + ex.getMessage(), true);
    }
    NamedCommands.registerCommand("TurnToTarget", new AimAtHub(m_swerve, joystick1, m_Lights));
    NamedCommands.registerCommand("ToggleShoot", new Shoot(m_Shooter, m_intake, m_swerveDrive));
    NamedCommands.registerCommand("ToggleIntake", m_intake.toggleIntake());
    NamedCommands.registerCommand("ToggleHopper", m_hopper.toggleHopperCommand());
    driveChooser.setDefaultOption("FieldOrientedDrive",Boolean.TRUE);
    driveChooser.addOption("RobotOrientedDrive",Boolean.FALSE);

    try {
          joystick1 = new CommandXboxController(Constants.controller.xboxPort1);
          xboxController = new CommandXboxController(Constants.controller.xboxPort2); // Creates an XboxController on port 2.
        } catch (RuntimeException ex) {
          DriverStation.reportError("Error instantiating Xboxcontroller: " + ex.getMessage(), true);
        }

    driveChooser.onChange((selectedOption)->{
      FieldOrientedDrive = selectedOption;
      System.out.println("field drive value"+selectedOption);
      if (FieldOrientedDrive){
        driveCmd = m_swerveDrive.fieldDriveCommand(
          () -> MathUtil.applyDeadband(-joystick1.getRawAxis(1), Constants.controller.LEFT_Y_DEADBAND), // X
          () -> MathUtil.applyDeadband(-joystick1.getRawAxis(0), Constants.controller.LEFT_X_DEADBAND), // Y 
          () -> -joystick1.getRawAxis(4),                                                               // Angle 1
          () -> -joystick1.getRawAxis(5)                                                              // Angle 2
          );
      }else {
       driveCmd = m_swerveDrive.robotDriveCommand(
          () -> MathUtil.applyDeadband(-joystick1.getRawAxis(1), Constants.controller.LEFT_Y_DEADBAND), // X
          () -> MathUtil.applyDeadband(-joystick1.getRawAxis(0), Constants.controller.LEFT_X_DEADBAND), // Y 
          () -> -joystick1.getRawAxis(4),                                                               // Angle 1
          () -> -joystick1.getRawAxis(5)                                                              // Angle 2
          );
        /*
        driveCmd = m_swerveDrive.fieldDriveCommand( 
        () -> MathUtil.applyDeadband(-joystick1.getRawAxis(1), Constants.controller.LEFT_Y_DEADBAND), // X
        () -> MathUtil.applyDeadband(-joystick1.getRawAxis(0), Constants.controller.LEFT_X_DEADBAND), // Y 
        () -> -joystick1.getRawAxis(4),                                                               // Angle 1
        () -> -joystick1.getRawAxis(5)
        ); */
      }
      
      
      
    m_swerveDrive.setDefaultCommand(driveCmd);
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
      // Perform actions based on the selected option
      });


    SmartDashboard.putData("Auto Chooser", autoChooser);
    SmartDashboard.putData("Drive Chooser", driveChooser);
    SmartDashboard.putData("Turn To Hub", new AimAtHub(m_swerve, joystick1, m_Lights));
    SmartDashboard.putData("drive",new AutoWaypoints(m_swerveDrive,  new Pose2d(3.235,7.186,Rotation2d.fromDegrees(-78.024))));
    SmartDashboard.putData("Blue Lights",m_Lights.blue());
    SmartDashboard.putData("Lights off",m_Lights.off());
    SmartDashboard.putData("Red Lights",m_Lights.red());
    SmartDashboard.putData("Rainbow Lights",m_Lights.rainbow());
    SmartDashboard.putData("Left",new AutoWaypoints(m_swerveDrive, new Pose2d(3.235,7.186,Rotation2d.fromDegrees(-78.024))));
    SmartDashboard.putData("Up", new AutoWaypoints(m_swerveDrive, new Pose2d(2.847,4.019,Rotation2d.fromDegrees(0))));
    SmartDashboard.putData("Down",new AutoWaypoints(m_swerveDrive, new Pose2d(1.804,3.965,Rotation2d.fromDegrees(0))));
    SmartDashboard.putData("Right", new AutoWaypoints(m_swerveDrive, new Pose2d(2.901,0.963,Rotation2d.fromDegrees(47.545))));
    SmartDashboard.putData("INTAKEtoggleIntake", m_intake.toggleIntake()); //new ParallelCommandGroup(m_intake.toggleIntake(), m_Shooter.toggleIntakeFeed(), m_Lights.red()));
    SmartDashboard.putData("INTAKEtoggleFeed", m_intake.toggleFeedMode()); 
    SmartDashboard.putData("INTAKEtoggleIntake", m_intake.toggleIntake()); 
    SmartDashboard.putData("ShootertoggleFeed", m_Shooter.toggleFeed());
    SmartDashboard.putData("ShooterFeedOn", m_Shooter.FeedOn());
    SmartDashboard.putData("ShooterFeedOff", m_Shooter.FeedOff());
    SmartDashboard.putData("ShootertoggleInakeFeed", m_Shooter.toggleIntakeFeed());
    SmartDashboard.putData("ShootertoggleShooter", m_Shooter.toggleShooter());
    SmartDashboard.putData("INTAKE ON", m_intake.IntakeOn());
    SmartDashboard.putData("INTAKE OFF", m_intake.IntakeOff());
    SmartDashboard.putData("Feed Off", m_intake.FeedOff());
    SmartDashboard.putData("Feed on", m_intake.FeedOn());
    SmartDashboard.putNumber("ShooterSpeed", 4200);
    SmartDashboard.putNumber("HoodAngle", 10);
    
    SmartDashboard.putData("Test Shoot", new testShoot(m_Shooter, m_intake, m_swerveDrive));
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
    joystick1.b().onTrue(new AimAtHub(m_swerve, joystick1, m_Lights));
    joystick1.back().onTrue( m_swerveDrive.ToggleBrake());
    joystick1.povLeft().onTrue(new AutoWaypoints(m_swerveDrive, new Pose2d(3.235,7.186,Rotation2d.fromDegrees(-78.024))));
    joystick1.povUp().onTrue(new AutoWaypoints(m_swerveDrive, new Pose2d(2.847,4.019,Rotation2d.fromDegrees(0))));
    joystick1.povDown().onTrue(new AutoWaypoints(m_swerveDrive, new Pose2d(1.804,3.965,Rotation2d.fromDegrees(0))));
    joystick1.povRight().onTrue(new AutoWaypoints(m_swerveDrive, new Pose2d(2.901,0.963,Rotation2d.fromDegrees(47.545))));
    joystick1.start().onTrue(new InstantCommand(m_swerveDrive::zeroGyro));
    
    /*Co-driver controls  Port 2 */
    //xboxController.povUp().onTrue( m_climber.Up());
    //xboxController.povDown().onTrue(m_climber.Down());
    xboxController.rightTrigger().onTrue(new SequentialCommandGroup(m_intake.toggleIntake(), m_Shooter.toggleIntakeFeed(), m_Lights.red()));
    xboxController.a().onTrue(m_hopper.toggleHopperCommand());
    xboxController.leftTrigger().onTrue(new Shoot(m_Shooter, m_intake, m_swerveDrive));
    xboxController.x().onTrue(m_Lights.pink());
    xboxController.povUp().onTrue(m_Shooter.adjustHoodup());
    xboxController.povDown().onTrue(m_Shooter.adjustHooddown());
    xboxController.b().onTrue(m_intake.toggleReverseIntake());
    }
    public void zeroTeleopFieldOrientation(){
      m_swerveDrive.zeroTeleopFieldOrientation();
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