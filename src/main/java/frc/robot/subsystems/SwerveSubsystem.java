// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.MetersPerSecond;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.swerve.SwerveSetpoint;
import com.pathplanner.lib.util.swerve.SwerveSetpointGenerator;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import frc.robot.Constants;
import frc.robot.subsystems.Vision.Cameras;
import java.io.File;
import java.io.IOException;
import java.util.Arrays;
import java.util.Optional;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.json.simple.parser.ParseException;
import org.photonvision.targeting.PhotonPipelineResult;
import swervelib.SwerveDrive;
import swervelib.SwerveDriveTest;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveControllerConfiguration;
import swervelib.parser.SwerveDriveConfiguration;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;


public class SwerveSubsystem extends SubsystemBase {
    private final SwerveDrive swerveDrive;
    public boolean brakeOn    = false;
    private Vision vision;
    private double maxChassisVelocity;
    private double maxChassisAngularVelocity;
    public PathConstraints constraints;
    private AprilTagFieldLayout aprilTagFieldLayout;

    public SwerveSubsystem(File directory){
        /*Configure the telemetry before creating the swerve to avoid clutter.*/
        SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
        aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);

        try{
            swerveDrive = new SwerveParser(directory).createSwerveDrive(Constants.DriveTrain.maxSpeed);
        }catch(Exception e) {
            throw new RuntimeException(e);
        }

        maxChassisVelocity = swerveDrive.getMaximumChassisVelocity();
        maxChassisAngularVelocity = swerveDrive.getMaximumChassisAngularVelocity();
        swerveDrive.setCosineCompensator(false);
        swerveDrive.setAngularVelocityCompensation(true, true, 0.1);
        swerveDrive.setChassisDiscretization(true, 0.015);
        setupPhotonVision();
        setupPathPlanner();
        
        constraints = new PathConstraints(
        maxChassisVelocity, 3.0,
        maxChassisAngularVelocity, Units.degreesToRadians(720));

        Pose2d startingPose = isRedAlliance() ? new Pose2d(new Translation2d(Meter.of(1),
                                                                      Meter.of(4)),
                                                    Rotation2d.fromDegrees(0))
                                       : new Pose2d(new Translation2d(Meter.of(16),
                                                                      Meter.of(4)),
                                                    Rotation2d.fromDegrees(180));
    }

    
  

    public SwerveSubsystem(SwerveDriveConfiguration driveCfg, SwerveControllerConfiguration controllerCfg) {
    swerveDrive =
        new SwerveDrive(
            driveCfg,
            controllerCfg,
            Constants.DriveTrain.maxSpeed,
            new Pose2d(new Translation2d(Meter.of(3), Meter.of(3)), Rotation2d.fromDegrees(0)));
            setupPathPlanner();
            setupPhotonVision();
  }

  public void setupPhotonVision()
  {
    vision = new Vision(swerveDrive::getPose, swerveDrive.field);
  }

  public void setupPathPlanner(){
    RobotConfig config;
    try{
        config = RobotConfig.fromGUISettings();
        final boolean enableFeedforward = true;
        AutoBuilder.configure(
            this::getPose,
            this::resetOdometry,
            this::getRobotVelocity,
           (speedsRobotRelative, moduleFeedForwards) -> {
            if (enableFeedforward) {
              swerveDrive.drive(
                  speedsRobotRelative,
                  swerveDrive.kinematics.toSwerveModuleStates(speedsRobotRelative),
                  moduleFeedForwards.linearForces());
            } else {
              swerveDrive.setChassisSpeeds(speedsRobotRelative);
            }},
            new PPHolonomicDriveController(
                new PIDConstants(5,0,0),
                //Translation Constants
                new PIDConstants(5,0,0)
                //Rotation Constants
            ),
            config,
            () -> {
                var alliance = DriverStation.getAlliance();
                if (alliance.isPresent()) {
                    return alliance.get() == DriverStation.Alliance.Red;
                }
                return false;
            },
            this);
            
    } catch (Exception e) {
      e.printStackTrace();
    }

    PathfindingCommand.warmupCommand().schedule();
  } //setupPathPlanner ends here

  public Command getAutonomousCommand(String pathName){
    return new PathPlannerAuto(pathName);
  }

  public Command driveFieldOriented(Supplier<ChassisSpeeds> velocity) {
    return run(
        () -> {
          swerveDrive.driveFieldOriented(velocity.get());
        });
  }

  public Command driveRobotOriented(Supplier<ChassisSpeeds> velocity) {
    return run(
        () -> {
          swerveDrive.setChassisSpeeds(velocity.get());
        });
  }
//VARIABLES DEFINED HERE
  public void drive(ChassisSpeeds velocity) {
    swerveDrive.drive(velocity);
  }

  public Pose2d getPose()
  {
    return swerveDrive.getPose();
  }
  
  public void resetOdometry(Pose2d initialHolonomicPose)
  {
    swerveDrive.resetOdometry(initialHolonomicPose);
  }

   public ChassisSpeeds getRobotVelocity()
  {
    return swerveDrive.getRobotVelocity();
  }

  public void zeroGyro() {
    swerveDrive.zeroGyro();
  }

   public void setMotorBrake(boolean brake) {
    swerveDrive.setMotorIdleMode(brake);
  }

  public Rotation2d getHeading() {
    return getPose().getRotation();
  }

  public SwerveDrive getSwerveDrive() {
    return swerveDrive;
  }

  public SwerveDriveKinematics getKinematics()
  {
    return swerveDrive.kinematics;
  }

  public void postTrajectory(Trajectory trajectory)
  {
    swerveDrive.postTrajectory(trajectory);
  }
  
  private void lock() {
        swerveDrive.lockPose();
    }

  public void brake()
  {
    brakeOn = !brakeOn;
    setMotorBrake(brakeOn);
    if (brakeOn) lock();
        System.out.println("Brake is " + (brakeOn ? "on" : "off") + ".");
      }

    public Command ToggleBrake() {
    return runOnce(this::brake);
    }
   
    public Rotation2d getPathVelocityHeading(ChassisSpeeds cs, Pose2d target)
  {
    if (getVelocityMagnitude(cs).in(MetersPerSecond) < 0.25) {
      var diff = target.minus(getPose()).getTranslation();
      return (diff.getNorm() < 0.01) ? target.getRotation() : diff.getAngle();
    }
    return Rotation2d.fromRadians(Math.atan2(cs.vyMetersPerSecond, cs.vxMetersPerSecond));
  }

   public LinearVelocity getVelocityMagnitude(ChassisSpeeds cs)
  {
    return MetersPerSecond.of(
        new Translation2d(cs.vxMetersPerSecond, cs.vyMetersPerSecond).getNorm());
  }

  
      //This is how 614 did their simulationPeriodic, I decided our way looked better.
      @Override
      public void simulationPeriodic(){
      Optional<Pose2d> simPoseOptional = swerveDrive.getSimulationDriveTrainPose();
      if(simPoseOptional.isPresent()) {
        Pose2d simPose = simPoseOptional.get();
        addVisionMeasurement(simPose, Timer.getFPGATimestamp());
        Pose2d odometryPose = getPose();
        double error = odometryPose.getTranslation().getDistance(simPose.getTranslation());
        }
      }
    

    public ChassisSpeeds getFieldVelocity()
    {
      return swerveDrive.getFieldVelocity();
    }

    public Command drivePath(PathPlannerPath path)
    {
      return AutoBuilder.followPath(path);
    }


    public boolean isRedAlliance()
    {
      var alliance = DriverStation.getAlliance();
      return alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;
    }  
   /*   
    @Override
  public void simulationPeriodic() {
    vision.simulationPeriodic(swerveDrive.getPose());
    var debugField = vision.getSimDebugField();
    debugField.getObject("EstimatedRobot").setPose(swerveDrive.getPose());
    if (vision != null) {
    vision.simulationPeriodic(swerveDrive.getPose());
    }
  } */

  public void addVisionMeasurement(Pose2d visionPose, double timestampSeconds){
    swerveDrive.addVisionMeasurement(visionPose, timestampSeconds);
  }

public ChassisSpeeds getTargetSpeeds(double xInput, double yInput,
                                       double headingX, double headingY) {
    Translation2d scaledInputs = SwerveMath.cubeTranslation(new Translation2d(xInput, yInput));
    return swerveDrive.swerveController.getTargetSpeeds(
        scaledInputs.getX(), scaledInputs.getY(),
        headingX, headingY,
        getHeading().getRadians(),
        Constants.DriveTrain.maxSpeed);
  }

public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, Rotation2d angle)
  {
    Translation2d scaledInputs = SwerveMath.cubeTranslation(new Translation2d(xInput, yInput));
    return swerveDrive.swerveController.getTargetSpeeds(
        scaledInputs.getX(), scaledInputs.getY(),
        angle.getRadians(),
        getHeading().getRadians(),
        Constants.DriveTrain.maxSpeed);
  }


public Command aimAtTarget(Cameras camera)
  {
    return run(() -> {
      Optional<PhotonPipelineResult> resultO = camera.getBestResult();
      if (resultO.isPresent()) {
        var result = resultO.get();
        if (result.hasTargets()) {
          drive(getTargetSpeeds(0, 0,
                                          Rotation2d.fromDegrees(result.getBestTarget().getYaw())));
                  }
                }
              });
            }
Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Red);

public DriverStation.Alliance getAlliance() {
    return DriverStation.getAlliance()
        .orElse(DriverStation.Alliance.Blue);
}

/** Get distance to the hub (alliance-aware). */
  public double DistancetoHub() {
    // Select hub tag by alliance
    int targetTag = (alliance == Alliance.Blue) ? 26 : 10;
    return getDistanceToTag(targetTag);
  }

  public double getDistanceToTag(int id) {
    return vision.getDistanceFromAprilTag(id);
  }

   public Pose2d GetTargetTag() {
    PhotonPipelineResult results = Cameras.FRONT_CAM.camera.getLatestResult();
    if (results.hasTargets()) {
      var tagPose = Cameras.FRONT_CAM.poseEstimator.getFieldTags()
                                                   .getTagPose(results.getBestTarget().getFiducialId());
      if (tagPose.isPresent()) {
        return tagPose.get().toPose2d();
      }
    }
        return swerveDrive.getPose();
    }

     public Pose2d GetTagPose(int tag){
    return aprilTagFieldLayout.getTagPose(tag)
        .orElseThrow(() -> new RuntimeException("AprilTag " + tag + " not found"))
        .toPose2d();
  }

   public Command driveToPose(Pose2d pose)
  {
    return AutoBuilder.pathfindToPose(pose, constraints, MetersPerSecond.of(0));
  }

  public Pose2d CoordinateConversion(Pose2d pointInRobotCoords)
  {
    return getPose().transformBy(
        new Transform2d(pointInRobotCoords.getTranslation(), pointInRobotCoords.getRotation()));
  }

  public Command driveToPoseRobotRelative(Pose2d robotRelativePose)
  {
    return driveToPose(CoordinateConversion(robotRelativePose));
  }

  private Command driveWithSetpointGenerator(Supplier<ChassisSpeeds> robotRelativeChassisSpeed)
      throws IOException, ParseException
  {
    SwerveSetpointGenerator setpointGenerator = new SwerveSetpointGenerator(
        RobotConfig.fromGUISettings(), maxChassisAngularVelocity);
    AtomicReference<SwerveSetpoint> prevSetpoint = new AtomicReference<>(
        new SwerveSetpoint(swerveDrive.getRobotVelocity(),
                           swerveDrive.getStates(),
                           DriveFeedforwards.zeros(swerveDrive.getModules().length)));
    AtomicReference<Double> previousTime = new AtomicReference<>();

    return startRun(
        () -> previousTime.set(Timer.getFPGATimestamp()),
        () -> {
          double newTime   = Timer.getFPGATimestamp();
          SwerveSetpoint newSetpoint = setpointGenerator.generateSetpoint(
              prevSetpoint.get(), robotRelativeChassisSpeed.get(), newTime - previousTime.get());
          swerveDrive.drive(newSetpoint.robotRelativeSpeeds(),
                            newSetpoint.moduleStates(),
                            newSetpoint.feedforwards().linearForces());
          prevSetpoint.set(newSetpoint);
          previousTime.set(newTime);
        });
  }

  public Command driveWithSetpointGeneratorFieldRelative(Supplier<ChassisSpeeds> fieldRelativeSpeeds)
  {
    try {
      return driveWithSetpointGenerator(
          () -> ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds.get(), getHeading()));
    } catch (Exception e) {
      DriverStation.reportError(e.toString(), true);
    }
    return Commands.none();
  }

  public Command sysIdDriveMotorCommand()
  {
    return SwerveDriveTest.generateSysIdCommand(
        SwerveDriveTest.setDriveSysIdRoutine(new Config(), this, swerveDrive, 12, false),
        3.0, 5.0, 3.0);
  }

  public Command sysIdAngleMotorCommand()
  {
    return SwerveDriveTest.generateSysIdCommand(
        SwerveDriveTest.setAngleSysIdRoutine(new Config(), this, swerveDrive),
        3.0, 5.0, 3.0);
  }

  public Command centerModulesCommand()
  {
    return run(() -> Arrays.asList(swerveDrive.getModules()).forEach(it -> it.setAngle(0.0)));
  }

  public Command driveToDistanceCommand(double distanceInMeters, double speedInMetersPerSecond)
  {
    // Use getNorm() on the translation instead of getDistance(new Translation2d(0,0))
    return run(() -> drive(new ChassisSpeeds(speedInMetersPerSecond, 0, 0)))
        .until(() -> swerveDrive.getPose().getTranslation().getNorm() > distanceInMeters);
  }

  public void replaceSwerveModuleFeedforward(double kS, double kV, double kA)
  {
    swerveDrive.replaceSwerveModuleFeedforward(new SimpleMotorFeedforward(kS, kV, kA));
  }

  public Command driveRobotRelative(DoubleSupplier translationX, DoubleSupplier translationY,
                                    DoubleSupplier angularRotationX)
  {
    return run(() -> swerveDrive.drive(
        SwerveMath.scaleTranslation(new Translation2d(
            translationX.getAsDouble() * maxChassisVelocity,
            translationY.getAsDouble() * maxChassisVelocity), 0.8),
        angularRotationX.getAsDouble() * maxChassisAngularVelocity,
        false, false));
  }

  public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY,
                              DoubleSupplier angularRotationX)
  {
    return run(() -> swerveDrive.drive(
        SwerveMath.scaleTranslation(new Translation2d(
            translationX.getAsDouble() * maxChassisVelocity,
            translationY.getAsDouble() * maxChassisVelocity), 0.8),
        Math.pow(angularRotationX.getAsDouble(), 3) * maxChassisAngularVelocity,
        true, false));
  }

  public Command robotDriveCommand(DoubleSupplier translationX, DoubleSupplier translationY,
                              DoubleSupplier headingX, DoubleSupplier headingY) {
    return run(() -> {
      Translation2d scaledInputs = SwerveMath.scaleTranslation(
          new Translation2d(isRedAlliance()? -translationX.getAsDouble():-translationX.getAsDouble(), 
          isRedAlliance()?translationY.getAsDouble():-translationY.getAsDouble()), 0.9*maxChassisVelocity);
      drive(swerveDrive.swerveController.getRawTargetSpeeds(
          scaledInputs.getX(), scaledInputs.getY(),
          headingX.getAsDouble()*maxChassisAngularVelocity) );
    });
  }

  public Command fieldDriveCommand(
    DoubleSupplier translationX,
    DoubleSupplier translationY,
    DoubleSupplier headingX,
    DoubleSupplier headingY)
{
    return run(() -> {

        Translation2d translation = new Translation2d(
            translationX.getAsDouble(),
            translationY.getAsDouble());

        double headingXVal = headingX.getAsDouble();
        double headingYVal = headingY.getAsDouble();

        swerveDrive.driveFieldOriented(
            swerveDrive.swerveController.getTargetSpeeds(
                translation.getX(),
                translation.getY(),
                headingXVal,
                headingYVal,
                swerveDrive.getOdometryHeading().getRadians(),
                maxChassisVelocity
            )
        );

    });
}


  public void drive(Translation2d translation, double rotation, boolean fieldRelative)
  {
    swerveDrive.drive(translation, rotation, fieldRelative, false);
  }

  public void driveFieldOriented(ChassisSpeeds velocity)
  {
    swerveDrive.driveFieldOriented(velocity);
  }

  public Command simDriveCommand(DoubleSupplier translationX, DoubleSupplier translationY,
                                 DoubleSupplier rotationX, DoubleSupplier rotationY)
  {
    return run(() -> driveFieldOriented(swerveDrive.swerveController.getTargetSpeeds(
        translationX.getAsDouble(), translationY.getAsDouble(),
        rotationX.getAsDouble(), rotationY.getAsDouble(),
        swerveDrive.getOdometryHeading().getRadians(),
        Constants.DriveTrain.maxSpeed)));
  }


}