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
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.Vision.Cameras;
import java.io.File;
import java.util.List;
import java.util.Optional;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.photonvision.targeting.PhotonPipelineResult;
import swervelib.SwerveDrive;
import swervelib.math.SwerveMath;
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

  private void setupPhotonVision()
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
    SmartDashboard.putNumber("DistancetoHub", DistancetoHub());
  }

  @Override
  public void simulationPeriodic()
  {
    vision.simulationPeriodic(swerveDrive.getPose());
    var debugField = vision.getSimDebugField();
    debugField.getObject("EstimatedRobot").setPose(swerveDrive.getPose());
  }

  private void setupPathPlanner()
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

  /** Get distance to the hub (alliance-aware). */
  public double DistancetoHub()
  {
    // Select hub tag by alliance
    int targetTag = isRedAlliance() ? 10 : 26;
    return getDistanceToTag(targetTag);
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


  public Command driveToPose(Pose2d pose)
  {
    return AutoBuilder.pathfindToPose(pose, constraints, MetersPerSecond.of(0));
  }






  private void drive(ChassisSpeeds velocity)
  {
    swerveDrive.drive(velocity);
  }

  public Command robotDriveCommand(DoubleSupplier translationX, DoubleSupplier translationY,
                              DoubleSupplier headingX, DoubleSupplier headingY)
  {
    return run(() -> {
      Translation2d scaledInputs = SwerveMath.scaleTranslation(
          new Translation2d(translationX.getAsDouble(), translationY.getAsDouble()), 0.9*maxChassisVelocity);
      drive(swerveDrive.swerveController.getRawTargetSpeeds(
          scaledInputs.getX(), scaledInputs.getY(),
          headingX.getAsDouble()*maxChassisAngularVelocity) );
      

    });
  }


  public void driveFieldOriented(ChassisSpeeds velocity)
  {
    swerveDrive.driveFieldOriented(velocity);
  }


  /**
   * Primary teleop drive command. In field-oriented mode, the right stick sets the heading angle;
   * the robot rotates to face that direction. Heading correction is off since we handle it here.
   */
  public Command fieldDriveCommand(DoubleSupplier translationX, DoubleSupplier translationY,
                                 DoubleSupplier headingX, DoubleSupplier headingY)
  {
    
      return run(() -> {
        Translation2d scaledInputs = SwerveMath.scaleTranslation(
            new Translation2d(isRedAlliance()?-translationX.getAsDouble():translationX.getAsDouble(), 
            isRedAlliance()?-translationY.getAsDouble():translationY.getAsDouble()), 0.9);
        double hX = headingX.getAsDouble();
        double hY = headingY.getAsDouble();

        swerveDrive.setHeadingCorrection(false);
        double defaultAngle = swerveDrive.swerveController.lastAngleScalar;
        double angle = swerveDrive.swerveController.withinHypotDeadband(hX, hY)
                      ? defaultAngle
                      : Math.atan2(hX, hY);
        if(isRedAlliance()){
          angle += Math.PI;
        }

        driveFieldOriented(swerveDrive.swerveController.getTargetSpeeds(
            scaledInputs.getX(), scaledInputs.getY(),
            angle,
            swerveDrive.getOdometryHeading().getRadians(),
            maxChassisVelocity));

        swerveDrive.swerveController.lastAngleScalar = angle;
      });  
  }


  public void resetOdometry(Pose2d initialHolonomicPose)
  {
    swerveDrive.resetOdometry(initialHolonomicPose);
  }

  public Pose2d getPose()
  {
    return swerveDrive.getPose();
  }


  public void zeroTeleopFieldOrientation()
  {
    Rotation2d allianceHeading = isRedAlliance() ? Rotation2d.fromDegrees(180) : Rotation2d.fromDegrees(0);
    Pose2d currentPose = getPose(); 
    resetOdometry(new Pose2d(currentPose.getTranslation(), allianceHeading));
  }

  public Command zeroTeleopFieldOrientationCommand()
  {
    return runOnce(this::zeroTeleopFieldOrientation);
  }

  public boolean isRedAlliance()
  {
    var alliance = DriverStation.getAlliance();
    return alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;
  }


  public void setMotorBrake(boolean brake)
  {
    swerveDrive.setMotorIdleMode(brake);
  }

  public Rotation2d getHeading()
  {
    return getPose().getRotation();
  }


  public ChassisSpeeds getFieldVelocity()
  {
    return swerveDrive.getFieldVelocity();
  }

  public ChassisSpeeds getRobotVelocity()
  {
    return swerveDrive.getRobotVelocity();
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
