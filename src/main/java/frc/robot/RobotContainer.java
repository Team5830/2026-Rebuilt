// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;
import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.Aim;
import frc.robot.commands.AutoWaypoints;
import frc.robot.commands.FireOne;
import frc.robot.commands.Shoot;
import frc.robot.commands.testShoot;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveSubsystem;
import swervelib.SwerveInputStream;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  //private SwerveSubsystem m_swerve;
  private SwerveSubsystem m_swerve;
  private Shooter m_Shooter =  new Shooter();
  //private Climber m_climber = new Climber(); 
  private Hopper m_hopper   = new Hopper();
  private Intake m_intake   = new Intake();
  private Lights m_Lights = new Lights();
  Command driveCmd = null;

  private CommandXboxController joystick1, xboxController;
  SwerveInputStream driveAngle;
  final SendableChooser<Command> autoChooser;
  final SendableChooser<Boolean> driveChooser= new SendableChooser<>();
  boolean FieldOrientedDrive = false;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    m_swerve =  new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),"swerve"));
    try {
      joystick1 = new CommandXboxController(Constants.controller.xboxPort1);
      xboxController = new CommandXboxController(Constants.controller.xboxPort2); // Creates an XboxController on port 2.
    } catch (RuntimeException ex) {
      DriverStation.reportError("Error instantiating Xboxcontroller: " + ex.getMessage(), true);
    }
    NamedCommands.registerCommand("TurnToTarget", new  Aim(m_swerve, driveAngle, m_Lights, joystick1));
    NamedCommands.registerCommand("ToggleShoot", new Shoot(m_Shooter, m_intake, m_swerve));
    NamedCommands.registerCommand("ToggleIntake", m_intake.toggleIntake());
    NamedCommands.registerCommand("ToggleHopper", m_hopper.toggleHopperCommand());
    SwerveInputStream driveAngularVelocity =
      SwerveInputStream.of(
              m_swerve.getSwerveDrive(), () -> -joystick1.getRawAxis(1), () -> -joystick1.getRawAxis(0))
          .withControllerRotationAxis(() -> -joystick1.getRightX())
          .deadband(Constants.controller.DEADBAND)
          .scaleTranslation(0.95)
          .allianceRelativeControl(true);

      driveAngle = driveAngularVelocity.copy()
          .withControllerHeadingAxis(() ->joystick1.getRawAxis(4)*(m_swerve.isRedAlliance()?1:-1), () ->joystick1.getRawAxis(5)*(m_swerve.isRedAlliance()?1:-1))
          .headingWhile(true)
          .deadband(Constants.controller.DEADBAND)
          .scaleTranslation(0.95)
          .allianceRelativeControl(true);

      SwerveInputStream driveAngularVelocityKeyboard =
      SwerveInputStream.of(
              m_swerve.getSwerveDrive(), () -> -joystick1.getLeftY(), () -> -joystick1.getLeftX())
          .withControllerRotationAxis(() -> joystick1.getRawAxis(2))
          .deadband(Constants.controller.DEADBAND)
          .scaleTranslation(0.95)
          .allianceRelativeControl(true);
    driveChooser.setDefaultOption("FieldOrientedDrive",Boolean.TRUE);
    driveChooser.addOption("RobotOrientedDrive",Boolean.FALSE);
    driveChooser.onChange((selectedOption)->{
      FieldOrientedDrive = selectedOption;
      System.out.println("field drive value"+selectedOption);
      if (FieldOrientedDrive){
        Command driveFieldOrientedAngle = m_swerve.driveFieldOriented(driveAngle);
        m_swerve.setDefaultCommand(driveFieldOrientedAngle);
      }else {
        Command driveRobotOriented = m_swerve.driveRobotOriented(driveAngularVelocity);
        m_swerve.setDefaultCommand(driveRobotOriented);
      }
    });
    Command driveFieldOrientedAngle = m_swerve.driveFieldOriented(driveAngle);
        m_swerve.setDefaultCommand(driveFieldOrientedAngle);
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
      if (pathsInAuto == null){
        return;
      }
      // Perform actions based on the selected option
      Pose2d startingPose = new Pose2d(0,0,Rotation2d.kZero);
      if (!pathsInAuto.isEmpty()) {
        PathPlannerPath path0 = pathsInAuto.get(0);
        startingPose = new Pose2d(path0.getPoint(0).position, path0.getIdealStartingState().rotation());
      }
      m_swerve.resetOdometry(startingPose);
    });

    

    SmartDashboard.putData("Auto Chooser", autoChooser);
    SmartDashboard.putData("Drive Chooser", driveChooser);
    SmartDashboard.putData("Turn To Hub", new  Aim(m_swerve, driveAngle, m_Lights, joystick1));
    SmartDashboard.putData("drive",new AutoWaypoints(m_swerve,  new Pose2d(3.235,7.186,Rotation2d.fromDegrees(-78.024))));
    SmartDashboard.putData("Blue Lights",m_Lights.blue());
    SmartDashboard.putData("Lights off",m_Lights.off());
    SmartDashboard.putData("Red Lights",m_Lights.red());
    SmartDashboard.putData("Rainbow Lights",m_Lights.rainbow());
    SmartDashboard.putData("Left",new AutoWaypoints(m_swerve, new Pose2d(3.235,7.186,Rotation2d.fromDegrees(-78.024))));
    SmartDashboard.putData("Up", new AutoWaypoints(m_swerve, new Pose2d(2.847,4.019,Rotation2d.fromDegrees(0))));
    SmartDashboard.putData("Down",new AutoWaypoints(m_swerve, new Pose2d(1.804,3.965,Rotation2d.fromDegrees(0))));
    SmartDashboard.putData("Right", new AutoWaypoints(m_swerve, new Pose2d(2.901,0.963,Rotation2d.fromDegrees(47.545))));
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
    
    SmartDashboard.putData("Test Shoot", new testShoot(m_Shooter, m_intake, m_swerve));
    //SmartDashboard.putData("Keys Test", m_Shooter.KeysToTheKingdomtest());
    SmartDashboard.putData("FireOneRepeat", new FireOne(m_Shooter).repeatedly());
    SmartDashboard.putData("FireOne", new FireOne(m_Shooter));
    SmartDashboard.putData("Cancel Shooter", Commands.runOnce(() -> {}, m_Shooter));
    //SmartDashboard.putData("GateOpen", new InstantCommand(m_Shooter.GateOpen()));
    SmartDashboard.putData("GateReject", m_Shooter.GateRejectToggle());
    SmartDashboard.putData("KeysKingdon",new SequentialCommandGroup(m_Shooter.ShootOn(), m_Shooter.KeysToTheKingdomtoggle()));
  
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
    joystick1.b().onTrue(new Aim(m_swerve, driveAngle, m_Lights, joystick1));
    joystick1.back().onTrue( m_swerve.ToggleBrake());
    joystick1.povLeft().onTrue(new AutoWaypoints(m_swerve, new Pose2d(3.235,7.186,Rotation2d.fromDegrees(-78.024))));
    joystick1.povUp().onTrue(new AutoWaypoints(m_swerve, new Pose2d(2.847,4.019,Rotation2d.fromDegrees(0))));
    joystick1.povDown().onTrue(new AutoWaypoints(m_swerve, new Pose2d(1.804,3.965,Rotation2d.fromDegrees(0))));
    joystick1.povRight().onTrue(new AutoWaypoints(m_swerve, new Pose2d(2.901,0.963,Rotation2d.fromDegrees(47.545))));
    joystick1.start().onTrue(new InstantCommand(m_swerve::zeroGyro));
    
    /*Co-driver controls  Port 2 */
    // xboxController.povUp().onTrue( m_climber.Up());
    // xboxController.povDown().onTrue(m_climber.Down());
    xboxController.rightTrigger().onTrue(new SequentialCommandGroup(m_intake.toggleIntake(), m_Shooter.toggleIntakeFeed(),m_Shooter.GateRejectToggle(), m_Lights.red()));
    xboxController.a().onTrue(m_hopper.toggleHopperCommand());
    xboxController.leftTrigger().onTrue(new Shoot(m_Shooter, m_intake, m_swerve));
    xboxController.x().onTrue(m_Lights.pink());
    xboxController.povUp().onTrue(m_Shooter.adjustHoodup());
    xboxController.povDown().onTrue(m_Shooter.adjustHooddown());
    xboxController.b().onTrue(m_intake.toggleReverseIntake());
    }
    
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */

  public Command getAutonomousCommand() {

    Command selected = autoChooser.getSelected();
    return selected;
    
  }
}