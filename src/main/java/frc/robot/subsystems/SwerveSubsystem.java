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
import edu.wpi.first.math.geometry.Transform3d;


public class SwerveSubsystem extends SubsystemBase
{
  private final SwerveDrive         swerveDrive;
  private final AprilTagFieldLayout aprilTagFieldLayout;
  private final boolean             visionDriveTest = true;
  private       Vision              vision;

  public boolean driveField   = false;
  public boolean brakeOn      = false;
  public PathConstraints constraints;
  public double  distancetohub = 0.0;

  // Alliance-specific hub tag IDs
  private static final int HUB_TAG_BLUE = 26;
  private static final int HUB_TAG_RED  = 10;

  /**
   * Initialize {@link SwerveDrive} with the directory provided.
   *
   * @param directory Directory of swerve drive config files.
   */
  public SwerveSubsystem(File directory)
  {
    aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);

    double angleConversionFactor = SwerveMath.calculateDegreesPerSteeringRotation(Constants.DriveTrain.angleGearRatio);
    double driveConversionFactor = SwerveMath.calculateMetersPerRotation(
        Constants.DriveTrain.wheelDiameterInches, Constants.DriveTrain.driveGearRatio);

    System.out.printf("\"conversionFactors\": { \"angle\": {\"factor\": %f }, \"drive\": {\"factor\": %f } }%n",
        angleConversionFactor, driveConversionFactor);
    System.out.println("Directory: " + directory);

    SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;

    try
    {
      Pose2d startPose = isRedAlliance()
          ? new Pose2d(new Translation2d(Meter.of(-0.7), Meter.of(-7.3)), Rotation2d.fromDegrees(0))
          : new Pose2d(new Translation2d(Meter.of(0.7),  Meter.of(7.3)),  Rotation2d.fromDegrees(0));

      swerveDrive = new SwerveParser(directory).createSwerveDrive(Constants.DriveTrain.maxSpeed, startPose);
    } catch (Exception e) {
      throw new RuntimeException(e);
    }

    swerveDrive.setHeadingCorrection(false);
    swerveDrive.setCosineCompensator(!SwerveDriveTelemetry.isSimulation);
    swerveDrive.setAngularVelocityCompensation(true, true, 0.1);
    swerveDrive.setModuleEncoderAutoSynchronize(false, 1);

    if (visionDriveTest && !RobotBase.isSimulation())
    {
      swerveDrive.stopOdometryThread();
    }

    setupPhotonVision();
    setupPathPlanner();

    constraints = new PathConstraints(
        swerveDrive.getMaximumChassisVelocity(), 3.0,
        swerveDrive.getMaximumChassisAngularVelocity(), Units.degreesToRadians(720));
  }

  /** Setup the PhotonVision pose estimator. */
  public void setupPhotonVision()
  {
    vision = new Vision(swerveDrive::getPose, swerveDrive.field);
  }

  @Override
  public void periodic()
  {
    swerveDrive.updateOdometry();
    if (visionDriveTest && !RobotBase.isSimulation())
    {
      vision.updatePoseEstimation(swerveDrive);
    }
    distancetohub = DistancetoHub();
  }

  @Override
  public void simulationPeriodic()
  {
    vision.simulationPeriodic(swerveDrive.getPose());
    var debugField = vision.getSimDebugField();
    debugField.getObject("EstimatedRobot").setPose(swerveDrive.getPose());
  }

  /** Setup AutoBuilder for PathPlanner. */
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
            if (enableFeedforward)
            {
              swerveDrive.drive(
                  speedsRobotRelative,
                  swerveDrive.kinematics.toSwerveModuleStates(speedsRobotRelative),
                  moduleFeedForwards.linearForces());
            } else
            {
              swerveDrive.setChassisSpeeds(speedsRobotRelative);
            }
          },
          new PPHolonomicDriveController(
              new PIDConstants(5.0, 0.0, 0.0),
              new PIDConstants(5.0, 0.0, 0.0)),
          config,
          () -> {
            Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
            return alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;
          },
          this);

    } catch (Exception e) {
      e.printStackTrace();
    }

    PathfindingCommand.warmupCommand().schedule();
  }

  /**
   * Aim the robot at the target returned by PhotonVision.
   *
   * @return A {@link Command} which will run the alignment.
   */
  public Command aimAtTarget(Cameras camera)
  {
    return run(() -> {
      Optional<PhotonPipelineResult> resultO = camera.getBestResult();
      if (resultO.isPresent())
      {
        var result = resultO.get();
        if (result.hasTargets())
        {
          drive(getTargetSpeeds(0, 0, Rotation2d.fromDegrees(result.getBestTarget().getYaw())));
        }
      }
    });
  }

  /** Get distance to the hub (alliance-aware). */
  public double DistancetoHub()
  {
    return getDistanceToTag(isRedAlliance() ? HUB_TAG_RED : HUB_TAG_BLUE);
  }

  public double getDistanceToTag(int id)
  {
    return vision.getDistanceFromAprilTag(id);
  }

  /**
   * Get the path follower with events.
   *
   * @param pathName PathPlanner path name.
   * @return {@link AutoBuilder#followPath(PathPlannerPath)} path command.
   */
  public Command getAutonomousCommand(String pathName)
  {
    return new PathPlannerAuto(pathName);
  }

  /**
   * Get the best-visible target tag pose from the front camera.
   * Returns the current robot pose if no target is visible.
   */
  public Pose2d GetTargetTag()
  {
    PhotonPipelineResult results = Cameras.FRONT_CAM.camera.getLatestResult();
    if (results.hasTargets())
    {
      var target  = results.getBestTarget();
      var tagPose = Cameras.FRONT_CAM.poseEstimator.getFieldTags()
          .getTagPose(target.getFiducialId());
      System.out.println("Target Acquired " + target.getFiducialId());
      if (tagPose.isPresent()) return tagPose.get().toPose2d();
    }
    return swerveDrive.getPose();
  }

  /**
   * Get a specific AprilTag's field pose.
   * Returns robot pose if tag is not in the layout (safe fallback).
   */
  public Pose2d GetTagPose(int tag)
  {
    return aprilTagFieldLayout.getTagPose(tag)
        .map(p -> p.toPose2d())
        .orElseGet(() -> {
          DriverStation.reportWarning("AprilTag " + tag + " not found in layout", false);
          return swerveDrive.getPose();
        });
  }

  /** Get the latest pipeline result from the front camera (empty result if no targets). */
  public PhotonPipelineResult GetTagResults()
  {
    PhotonPipelineResult result = Cameras.FRONT_CAM.camera.getLatestResult();
    return result.hasTargets() ? result : new PhotonPipelineResult();
  }

  /**
   * Use PathPlanner pathfinding to drive to a point on the field.
   *
   * @param pose Target {@link Pose2d} to go to.
   * @return PathFinding command
   */
  public Command driveToPose(Pose2d pose)
  {
    return AutoBuilder.pathfindToPose(pose, constraints, MetersPerSecond.of(0));
  }

  /** Convert a robot-relative pose to a field-relative pose. */
  public Pose2d CoordinateConversion(Pose2d pointInRobotCoords)
  {
    return getPose().transformBy(
        new Transform2d(pointInRobotCoords.getTranslation(), pointInRobotCoords.getRotation()));
  }

  public Command driveToPoseRobotRelative(Pose2d robotRelativePose)
  {
    return driveToPose(CoordinateConversion(robotRelativePose));
  }

  /**
   * Drive with {@link SwerveSetpointGenerator} from 254, implemented by PathPlanner.
   *
   * @param robotRelativeChassisSpeed Robot relative {@link ChassisSpeeds} to achieve.
   * @return {@link Command} to run.
   */
  private Command driveWithSetpointGenerator(Supplier<ChassisSpeeds> robotRelativeChassisSpeed)
  throws IOException, ParseException
  {
    SwerveSetpointGenerator setpointGenerator = new SwerveSetpointGenerator(
        RobotConfig.fromGUISettings(), swerveDrive.getMaximumChassisAngularVelocity());
    AtomicReference<SwerveSetpoint> prevSetpoint = new AtomicReference<>(
        new SwerveSetpoint(swerveDrive.getRobotVelocity(),
                           swerveDrive.getStates(),
                           DriveFeedforwards.zeros(swerveDrive.getModules().length)));
    AtomicReference<Double> previousTime = new AtomicReference<>();

    return startRun(
        () -> previousTime.set(Timer.getFPGATimestamp()),
        () -> {
          double newTime = Timer.getFPGATimestamp();
          SwerveSetpoint newSetpoint = setpointGenerator.generateSetpoint(
              prevSetpoint.get(), robotRelativeChassisSpeed.get(), newTime - previousTime.get());
          swerveDrive.drive(newSetpoint.robotRelativeSpeeds(),
                            newSetpoint.moduleStates(),
                            newSetpoint.feedforwards().linearForces());
          prevSetpoint.set(newSetpoint);
          previousTime.set(newTime);
        });
  }

  /**
   * Drive with 254's Setpoint generator using field-relative speeds.
   *
   * @param fieldRelativeSpeeds Field-relative {@link ChassisSpeeds}
   * @return Command to drive the robot using the setpoint generator.
   */
  public Command driveWithSetpointGeneratorFieldRelative(Supplier<ChassisSpeeds> fieldRelativeSpeeds)
  {
    try
    {
      return driveWithSetpointGenerator(
          () -> ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds.get(), getHeading()));
    } catch (Exception e) {
      DriverStation.reportError(e.toString(), true);
    }
    return Commands.none();
  }

  /** Command to characterize the robot drive motors using SysId. */
  public Command sysIdDriveMotorCommand()
  {
    return SwerveDriveTest.generateSysIdCommand(
        SwerveDriveTest.setDriveSysIdRoutine(new Config(), this, swerveDrive, 12, false),
        3.0, 5.0, 3.0);
  }

  /** Command to characterize the robot angle motors using SysId. */
  public Command sysIdAngleMotorCommand()
  {
    return SwerveDriveTest.generateSysIdCommand(
        SwerveDriveTest.setAngleSysIdRoutine(new Config(), this, swerveDrive),
        3.0, 5.0, 3.0);
  }

  /** Centers all swerve modules. */
  public Command centerModulesCommand()
  {
    return run(() -> Arrays.asList(swerveDrive.getModules()).forEach(it -> it.setAngle(0.0)));
  }

  /**
   * Drive the robot a fixed distance at a given speed.
   *
   * @param distanceInMeters       distance to travel
   * @param speedInMetersPerSecond travel speed
   */
  public Command driveToDistanceCommand(double distanceInMeters, double speedInMetersPerSecond)
  {
    return run(() -> drive(new ChassisSpeeds(speedInMetersPerSecond, 0, 0)))
        .until(() -> swerveDrive.getPose().getTranslation().getNorm() > distanceInMeters);
  }

  /**
   * Replaces the swerve module feedforward.
   *
   * @param kS static gain
   * @param kV velocity gain
   * @param kA acceleration gain
   */
  public void replaceSwerveModuleFeedforward(double kS, double kV, double kA)
  {
    swerveDrive.replaceSwerveModuleFeedforward(new SimpleMotorFeedforward(kS, kV, kA));
  }

  /**
   * Robot-relative drive command.
   *
   * @param translationX     X translation (–1 to 1)
   * @param translationY     Y translation (–1 to 1)
   * @param angularRotationX Angular velocity (–1 to 1)
   */
  public Command driveRobotRelative(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier angularRotationX)
  {
    return run(() ->
        swerveDrive.drive(
            SwerveMath.scaleTranslation(new Translation2d(
                translationX.getAsDouble() * swerveDrive.getMaximumChassisVelocity(),
                translationY.getAsDouble() * swerveDrive.getMaximumChassisVelocity()), 0.8),
            angularRotationX.getAsDouble() * swerveDrive.getMaximumChassisAngularVelocity(),
            false, false));
  }

  /**
   * Field-oriented drive command with cubed angular velocity for smooth control.
   *
   * @param translationX     X translation (–1 to 1)
   * @param translationY     Y translation (–1 to 1)
   * @param angularRotationX Angular velocity (–1 to 1, cubed)
   */
  public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier angularRotationX)
  {
    return run(() ->
        swerveDrive.drive(
            SwerveMath.scaleTranslation(new Translation2d(
                translationX.getAsDouble() * swerveDrive.getMaximumChassisVelocity(),
                translationY.getAsDouble() * swerveDrive.getMaximumChassisVelocity()), 0.8),
            Math.pow(angularRotationX.getAsDouble(), 3) * swerveDrive.getMaximumChassisAngularVelocity(),
            true, false));
  }

  /**
   * Field-oriented heading-angle drive command (heading set by right joystick direction).
   *
   * @param translationX X translation (–1 to 1)
   * @param translationY Y translation (–1 to 1)
   * @param headingX     Right-stick X
   * @param headingY     Right-stick Y
   */
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
          swerveDrive.getMaximumChassisVelocity()));
    });
  }

  /**
   * The primary method for controlling the drivebase.
   *
   * @param translation   Commanded linear velocity (m/s). Positive X = forward, positive Y = left.
   * @param rotation      Angular rate (rad/s). CCW positive.
   * @param fieldRelative True for field-relative, false for robot-relative.
   */
  public void drive(Translation2d translation, double rotation, boolean fieldRelative)
  {
    swerveDrive.drive(translation, rotation, fieldRelative, false);
  }

  /** Drive the robot using a field-oriented ChassisSpeeds. */
  public void driveFieldOriented(ChassisSpeeds velocity)
  {
    swerveDrive.driveFieldOriented(velocity);
  }

  /** Drive the robot using a field-oriented ChassisSpeeds supplier (for use in commands). */
  public Command driveFieldOriented(Supplier<ChassisSpeeds> velocity)
  {
    return run(() -> swerveDrive.driveFieldOriented(velocity.get()));
  }

  /** Drive according to robot-oriented ChassisSpeeds. */
  public void drive(ChassisSpeeds velocity)
  {
    swerveDrive.drive(velocity);
  }

  /**
   * Simulation-only drive command using heading-angle control.
   *
   * @param translationX X translation
   * @param translationY Y translation
   * @param rotationX    Heading X
   * @param rotationY    Heading Y
   */
  public Command simDriveCommand(DoubleSupplier translationX, DoubleSupplier translationY,
                                 DoubleSupplier rotationX, DoubleSupplier rotationY)
  {
    return run(() ->
        driveFieldOriented(swerveDrive.swerveController.getTargetSpeeds(
            translationX.getAsDouble(), translationY.getAsDouble(),
            rotationX.getAsDouble(), rotationY.getAsDouble(),
            swerveDrive.getOdometryHeading().getRadians(),
            Constants.DriveTrain.maxSpeed)));
  }

  /**
   * Primary teleop drive command.
   * Uses heading-angle control: the right stick sets the robot's facing direction.
   * Falls back to the last held angle when the stick is inside the deadband.
   *
   * @param translationX Left-stick X
   * @param translationY Left-stick Y
   * @param headingX     Right-stick X
   * @param headingY     Right-stick Y
   */
  public Command oneDriveCommand(DoubleSupplier translationX, DoubleSupplier translationY,
                                 DoubleSupplier headingX, DoubleSupplier headingY)
  {
    return run(() -> {
      Translation2d scaledInputs = SwerveMath.scaleTranslation(
          new Translation2d(translationX.getAsDouble(), translationY.getAsDouble()), 0.8);

      swerveDrive.setHeadingCorrection(false);

      double hx           = headingX.getAsDouble();
      double hy           = headingY.getAsDouble();
      double defaultAngle = swerveDrive.swerveController.lastAngleScalar;
      double angle        = swerveDrive.swerveController.withinHypotDeadband(hx, hy)
          ? defaultAngle
          : Math.atan2(hx, hy);

      driveFieldOriented(swerveDrive.swerveController.getTargetSpeeds(
          scaledInputs.getX(), scaledInputs.getY(),
          angle,
          swerveDrive.getOdometryHeading().getRadians(),
          swerveDrive.getMaximumChassisVelocity()));

      swerveDrive.swerveController.lastAngleScalar = angle;
    });
  }

  /** Get the swerve drive kinematics object. */
  public SwerveDriveKinematics getKinematics()
  {
    return swerveDrive.kinematics;
  }

  /** Resets odometry to the given pose. */
  public void resetOdometry(Pose2d initialHolonomicPose)
  {
    swerveDrive.resetOdometry(initialHolonomicPose);
  }

  /** Gets the current robot pose from odometry. */
  public Pose2d getPose()
  {
    return swerveDrive.getPose();
  }

  /** Set chassis speeds with closed-loop velocity control. */
  public void setChassisSpeeds(ChassisSpeeds chassisSpeeds)
  {
    swerveDrive.setChassisSpeeds(chassisSpeeds);
  }

  /** Post a trajectory to the field widget. */
  public void postTrajectory(Trajectory trajectory)
  {
    swerveDrive.postTrajectory(trajectory);
  }

  /** Resets the gyro angle to zero and resets odometry facing forward. */
  public void zeroGyro()
  {
    swerveDrive.zeroGyro();
  }

  /**
   * Checks if the alliance is red.
   *
   * @return true if red alliance, false otherwise (defaults to false if unavailable).
   */
  public boolean isRedAlliance()
  {
    Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
    return alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;
  }

  /**
   * Zero the gyro with alliance awareness.
   * Red alliance additionally resets pose to 180° to account for mirrored field orientation.
   */
  public void zeroGyroWithAlliance()
  {
    zeroGyro();
    if (isRedAlliance())
    {
      resetOdometry(new Pose2d(getPose().getTranslation(), Rotation2d.fromDegrees(180)));
    }
  }

  /**
   * Sets the drive motors to brake/coast mode.
   *
   * @param brake True for brake mode, false for coast.
   */
  public void setMotorBrake(boolean brake)
  {
    swerveDrive.setMotorIdleMode(brake);
  }

  /** Gets the current yaw from the pose estimator. */
  public Rotation2d getHeading()
  {
    return getPose().getRotation();
  }

  /**
   * Get chassis speeds from joystick input using heading-angle control.
   *
   * @param xInput   X joystick input
   * @param yInput   Y joystick input
   * @param headingX Heading X joystick value
   * @param headingY Heading Y joystick value
   */
  public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, double headingX, double headingY)
  {
    Translation2d scaledInputs = SwerveMath.cubeTranslation(new Translation2d(xInput, yInput));
    return swerveDrive.swerveController.getTargetSpeeds(
        scaledInputs.getX(), scaledInputs.getY(),
        headingX, headingY,
        getHeading().getRadians(),
        Constants.DriveTrain.maxSpeed);
  }

  /**
   * Get chassis speeds from joystick input toward a specific angle.
   *
   * @param xInput X joystick input
   * @param yInput Y joystick input
   * @param angle  Target heading angle
   */
  public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, Rotation2d angle)
  {
    Translation2d scaledInputs = SwerveMath.cubeTranslation(new Translation2d(xInput, yInput));
    return swerveDrive.swerveController.getTargetSpeeds(
        scaledInputs.getX(), scaledInputs.getY(),
        angle.getRadians(),
        getHeading().getRadians(),
        Constants.DriveTrain.maxSpeed);
  }

  /** Gets the current field-relative velocity. */
  public ChassisSpeeds getFieldVelocity()
  {
    return swerveDrive.getFieldVelocity();
  }

  /** Gets the current robot-relative velocity. */
  public ChassisSpeeds getRobotVelocity()
  {
    return swerveDrive.getRobotVelocity();
  }

  /** Alias for getRobotVelocity() — required by PathPlanner. */
  public ChassisSpeeds getRobotRelativeSpeeds()
  {
    return swerveDrive.getRobotVelocity();
  }

  /** Get the SwerveController. */
  public SwerveController getSwerveController()
  {
    return swerveDrive.swerveController;
  }

  /** Get the SwerveDriveConfiguration. */
  public SwerveDriveConfiguration getSwerveDriveConfiguration()
  {
    return swerveDrive.swerveDriveConfiguration;
  }

  /** Lock the swerve drive to prevent it from moving. */
  public void lock()
  {
    swerveDrive.lockPose();
  }

  /** Toggle brake on/off. */
  public Command ToggleBrake()
  {
    return runOnce(this::brake);
  }

  /** Internal brake toggle: locks + brake on, or releases both. */
  public void brake()
  {
    if (brakeOn) {
      setMotorBrake(false);
      System.out.println("Brake is off.");
      brakeOn = false;
    } else {
      setMotorBrake(true);
      lock();
      brakeOn = true;
      System.out.println("Brake is on.");
    }
  }

  /** Gets the current pitch angle of the robot. */
  public Rotation2d getPitch()
  {
    return swerveDrive.getPitch();
  }

  /** Add a fake vision reading for testing. */
  public void addFakeVisionReading()
  {
    swerveDrive.addVisionMeasurement(new Pose2d(3, 3, Rotation2d.fromDegrees(65)), Timer.getFPGATimestamp());
  }

  /**
   * Get the initial heading for a path toward a waypoint.
   * Uses velocity direction when moving fast enough; falls back to position delta otherwise.
   */
  public Rotation2d getPathVelocityHeading(ChassisSpeeds cs, Pose2d target)
  {
    if (getVelocityMagnitude(cs).in(MetersPerSecond) < 0.25)
    {
      var diff = target.minus(getPose()).getTranslation();
      return (diff.getNorm() < 0.01) ? target.getRotation() : diff.getAngle();
    }
    return new Rotation2d(cs.vxMetersPerSecond, cs.vyMetersPerSecond);
  }

  /** Get the scalar speed magnitude from ChassisSpeeds. */
  public LinearVelocity getVelocityMagnitude(ChassisSpeeds cs)
  {
    return MetersPerSecond.of(new Translation2d(cs.vxMetersPerSecond, cs.vyMetersPerSecond).getNorm());
  }

  /** Gets the underlying SwerveDrive object. */
  public SwerveDrive getSwerveDrive()
  {
    return swerveDrive;
  }

  /**
   * Build and follow a PathPlanner path to the given waypoint from the robot's current position.
   *
   * @param waypoint Target pose.
   */
  public Command getPathFromWaypoint(Pose2d waypoint)
  {
    List waypoints = PathPlannerPath.waypointsFromPoses(
        new Pose2d(getPose().getTranslation(), getPathVelocityHeading(getFieldVelocity(), waypoint)),
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
