// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.MetersPerSecond;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.FlippingUtil;
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
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import frc.robot.Constants;
import frc.robot.subsystems.Vision.Cameras;
import java.io.File;
import java.io.IOException;
import java.util.Arrays;
import java.util.List;
import java.util.Optional;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.json.simple.parser.ParseException;
import org.photonvision.targeting.PhotonPipelineResult;
import swervelib.SwerveController;
import swervelib.SwerveDrive;
import swervelib.SwerveDriveTest;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveDriveConfiguration;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;


public class SwerveSubsystem extends SubsystemBase
{
  private final SwerveDrive         swerveDrive;
  private final AprilTagFieldLayout aprilTagFieldLayout;
  private final boolean             visionDriveTest = true;
  private       Vision              vision;

  public boolean driveField = false;
  public boolean brakeOn    = false;
  public PathConstraints constraints;

  // Cached to avoid repeated calls in hot loops (periodic / drive commands)
  private final double maxChassisVelocity;
  private final double maxChassisAngularVelocity;

  public SwerveSubsystem(File directory)
  {
    aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);
    SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;

    try
    {
      swerveDrive = new SwerveParser(directory).createSwerveDrive(
          Constants.DriveTrain.maxSpeed,
          new Pose2d(new Translation2d(Meter.of(0.7), Meter.of(7.3)), Rotation2d.fromDegrees(0)));
    } catch (Exception e) {
      throw new RuntimeException(e);
    }

    maxChassisVelocity        = swerveDrive.getMaximumChassisVelocity();
    maxChassisAngularVelocity = swerveDrive.getMaximumChassisAngularVelocity();

    swerveDrive.setHeadingCorrection(false);
    swerveDrive.setCosineCompensator(!SwerveDriveTelemetry.isSimulation);
    swerveDrive.setAngularVelocityCompensation(true, true, 0.1);
    swerveDrive.setModuleEncoderAutoSynchronize(false, 1);

    if (visionDriveTest && !RobotBase.isSimulation()) {
      swerveDrive.stopOdometryThread();
    }
    setupPhotonVision();
    setupPathPlanner();

    constraints = new PathConstraints(
        maxChassisVelocity, 3.0,
        maxChassisAngularVelocity, Units.degreesToRadians(720));
  }

  public void setupPhotonVision()
  {
    vision = new Vision(swerveDrive::getPose, swerveDrive.field);
  }

  @Override
  public void periodic()
  {
    swerveDrive.updateOdometry();
    if (visionDriveTest && !RobotBase.isSimulation()) {
      vision.updatePoseEstimation(swerveDrive);
    }
  }

  @Override
  public void simulationPeriodic()
  {
    vision.simulationPeriodic(swerveDrive.getPose());
    var debugField = vision.getSimDebugField();
    debugField.getObject("EstimatedRobot").setPose(swerveDrive.getPose());
  }

  public void setupPathPlanner()
  {
    try
    {
      RobotConfig config = RobotConfig.fromGUISettings();
      final boolean enableFeedforward = false;

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
            }
          },
          new PPHolonomicDriveController(
              new PIDConstants(5.0, 0.0, 0.0),
              new PIDConstants(5.0, 0.0, 0.0)),
          config,
          // Return true when on red alliance so PathPlanner mirrors paths
          () -> {
            var alliance = DriverStation.getAlliance();
            return alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;
          },
          this);

    } catch (Exception e) {
      e.printStackTrace();
    }
    PathfindingCommand.warmupCommand().schedule();
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

  /** Get distance to the hub (alliance-aware). */
  public double DistancetoHub()
  {
    // Select hub tag by alliance
    int targetTag = isRedAlliance() ? 10 : 26;
    Pose2d tagpose   = GetTagPose(targetTag);
    Pose2d robotpose = getPose();
    double degreesToTurn = tagpose.minus(robotpose).getRotation().getDegrees();
    return driveToPose(new Pose2d(robotpose.getX(), robotpose.getY(),
                                  Rotation2d.fromDegrees(degreesToTurn)));
  }

  public double getDistanceToTag(int id)
  {
    return vision.getDistanceFromAprilTag(id);
  }

  public Command getAutonomousCommand(String pathName)
  {
    return new PathPlannerAuto(pathName);
  }

  /** Returns the pose of the best visible AprilTag, or current robot pose if none found. */
  public Pose2d GetTargetTag()
  {
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

  public Pose2d GetTagPose(int tag)
  {
    return aprilTagFieldLayout.getTagPose(tag)
                              .orElseThrow(() -> new RuntimeException("AprilTag " + tag + " not found"))
                              .toPose2d();
  }

  public PhotonPipelineResult GetTagResults()
  {
    PhotonPipelineResult result = Cameras.FRONT_CAM.camera.getLatestResult();
    return result.hasTargets() ? result : new PhotonPipelineResult();
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

  public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY,
                              DoubleSupplier headingX, DoubleSupplier headingY)
  {
    return run(() -> {
      Translation2d scaledInputs = SwerveMath.scaleTranslation(
          new Translation2d(translationX.getAsDouble(), translationY.getAsDouble()), 0.8);
      driveFieldOriented(swerveDrive.swerveController.getTargetSpeeds(
          scaledInputs.getX(), scaledInputs.getY(),
          headingX.getAsDouble(), headingY.getAsDouble(),
          swerveDrive.getOdometryHeading().getRadians(),
          maxChassisVelocity));
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

  public Command driveFieldOriented(Supplier<ChassisSpeeds> velocity)
  {
    return run(() -> swerveDrive.driveFieldOriented(velocity.get()));
  }

  public void drive(ChassisSpeeds velocity)
  {
    swerveDrive.drive(velocity);
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

  /**
   * Primary teleop drive command. In field-oriented mode, the right stick sets the heading angle;
   * the robot rotates to face that direction. Heading correction is off since we handle it here.
   */
  public Command oneDriveCommand(DoubleSupplier translationX, DoubleSupplier translationY,
                                 DoubleSupplier headingX, DoubleSupplier headingY)
  {
    return run(() -> {
      Translation2d scaledInputs = SwerveMath.scaleTranslation(
          new Translation2d(translationX.getAsDouble(), translationY.getAsDouble()), 0.8);
      double hX = headingX.getAsDouble();
      double hY = headingY.getAsDouble();

      swerveDrive.setHeadingCorrection(false);
      double defaultAngle = swerveDrive.swerveController.lastAngleScalar;
      double angle = swerveDrive.swerveController.withinHypotDeadband(hX, hY)
                     ? defaultAngle
                     : Math.atan2(hX, hY);

      driveFieldOriented(swerveDrive.swerveController.getTargetSpeeds(
          scaledInputs.getX(), scaledInputs.getY(),
          angle,
          swerveDrive.getOdometryHeading().getRadians(),
          maxChassisVelocity));

      swerveDrive.swerveController.lastAngleScalar = angle;
    });
  }

  public SwerveDriveKinematics getKinematics()
  {
    return swerveDrive.kinematics;
  }

  public void resetOdometry(Pose2d initialHolonomicPose)
  {
    swerveDrive.resetOdometry(initialHolonomicPose);
  }

  public Pose2d getPose()
  {
    return swerveDrive.getPose();
  }

  public void setChassisSpeeds(ChassisSpeeds chassisSpeeds)
  {
    swerveDrive.setChassisSpeeds(chassisSpeeds);
  }

  public void postTrajectory(Trajectory trajectory)
  {
    swerveDrive.postTrajectory(trajectory);
  }

  public void zeroGyro()
  {
    swerveDrive.zeroGyro();
  }

  private boolean isRedAlliance()
  {
    var alliance = DriverStation.getAlliance();
    return alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;
  }

  public void zeroGyroWithAlliance(Pose2d startingpose)
  {
    resetOdometry(isRedAlliance() ? FlippingUtil.flipFieldPose(startingpose) : startingpose);
  }

  public void setMotorBrake(boolean brake)
  {
    swerveDrive.setMotorIdleMode(brake);
  }

  public Rotation2d getHeading()
  {
    return getPose().getRotation();
  }

  public ChassisSpeeds getTargetSpeeds(double xInput, double yInput,
                                       double headingX, double headingY)
  {
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

  public ChassisSpeeds getFieldVelocity()
  {
    return swerveDrive.getFieldVelocity();
  }

  public ChassisSpeeds getRobotVelocity()
  {
    return swerveDrive.getRobotVelocity();
  }

  /** Alias of getRobotVelocity() kept for PathPlanner compatibility. */
  public ChassisSpeeds getRobotRelativeSpeeds()
  {
    return swerveDrive.getRobotVelocity();
  }

  public SwerveController getSwerveController()
  {
    return swerveDrive.swerveController;
  }

  public SwerveDriveConfiguration getSwerveDriveConfiguration()
  {
    return swerveDrive.swerveDriveConfiguration;
  }

  public void lock()
  {
    swerveDrive.lockPose();
  }

  public Command ToggleBrake()
  {
    return runOnce(this::brake);
  }

  /** Internal brake toggle: locks + brake on, or releases both. */
  public void brake()
  {
    brakeOn = !brakeOn;
    setMotorBrake(brakeOn);
    if (brakeOn) lock();
    System.out.println("Brake is " + (brakeOn ? "on" : "off") + ".");
  }

  public Rotation2d getPitch()
  {
    return swerveDrive.getPitch();
  }

  public void addFakeVisionReading()
  {
    swerveDrive.addVisionMeasurement(new Pose2d(3, 3, Rotation2d.fromDegrees(65)),
                                     Timer.getFPGATimestamp());
  }

  public Rotation2d getPathVelocityHeading(ChassisSpeeds cs, Pose2d target)
  {
    if (getVelocityMagnitude(cs).in(MetersPerSecond) < 0.25) {
      var diff = target.minus(getPose()).getTranslation();
      return (diff.getNorm() < 0.01) ? target.getRotation() : diff.getAngle();
    }
    return new Rotation2d(cs.vxMetersPerSecond, cs.vyMetersPerSecond);
  }

  public LinearVelocity getVelocityMagnitude(ChassisSpeeds cs)
  {
    return MetersPerSecond.of(
        new Translation2d(cs.vxMetersPerSecond, cs.vyMetersPerSecond).getNorm());
  }

  public SwerveDrive getSwerveDrive()
  {
    return swerveDrive;
  }

  public Command getPathFromWaypoint(Pose2d waypoint)
  {
    List waypoints = PathPlannerPath.waypointsFromPoses(
        new Pose2d(getPose().getTranslation(),
                   getPathVelocityHeading(getFieldVelocity(), waypoint)),
        waypoint);
    PathPlannerPath path = new PathPlannerPath(
        waypoints, constraints,
        new IdealStartingState(getVelocityMagnitude(getFieldVelocity()), getHeading()),
        new GoalEndState(0.0, waypoint.getRotation()));
    path.preventFlipping = true;
    return AutoBuilder.followPath(path);
  }

  /**
   * Follow a pre-built PathPlanner path.
   *
   * @param path The {@link PathPlannerPath} to follow.
   */
  public Command drivePath(PathPlannerPath path)
  {
    return AutoBuilder.followPath(path);
  }
}
