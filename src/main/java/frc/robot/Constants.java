// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.math.util.Units;
import static edu.wpi.first.units.Units.Centimeter;
import static edu.wpi.first.units.Units.Seconds;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;

import static edu.wpi.first.units.Units.InchesPerSecond;


/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class DriveTrain {
    public static final double maxSpeed = 3.0; // 4 meters per second
    public static final double wheelDiameterInches = 4.0;
    public static final double maxAcceleration = 1.0;
    public static final double maxAngularVelocity = 15; // revolutions per second
    public static final double maxAngularAcceleration = 20;
    public static final double wheelCircumferenceInches = wheelDiameterInches * Math.PI; // 12.5;
    public static final double driveGearRatio = 6.55; //Check Pinion: 10t, Central Gear: 42t:30t, Second Stage: 16t: total 6.72
    public static final double driveEncoderFactor = (wheelCircumferenceInches*2.54/100)/driveGearRatio; //Value used in JSON swerve config
    public static final double angleGearRatio = 13.37; 
    // //PIDS
    // public static final double driveControllerKp = 0.03;
    // public static final double driveControllerKi = 0;
    // public static final double driveControllerKd = 0.03;
    // public static final double turnControllerKp = .04;
    // public static final double turnControllerKi = 0;
    // public static final double turnControllerKd = 0.002;
  }
   public static final class controller {
    public static final int xboxPort1 = 0; //Driver
    public static final int xboxPort2 = 1; //Co -driver
    public static final double xRateLimit = 20;
    public static final double yRateLimit = 20;
    public static final double rotRateLimit = 2;
    public static final double LEFT_X_DEADBAND = 0.01;
    public static final double LEFT_Y_DEADBAND = 0.01;
  }
    public static class AutoAlign {
    public static final Time kAutoAlignPredict = Seconds.of(0.0);
    public static final Rotation2d kRotationTolerance = Rotation2d.fromDegrees(3.0);
    public static final Distance kPositionTolerance = Centimeter.of(1.5);
    public static final LinearVelocity kSpeedTolerance = InchesPerSecond.of(2);
    public static final Time kAutoAlignAdjustTimeout = Seconds.of(0.6);
    public static final Time kTeleopAlignAdjustTimeout = Seconds.of(2);
    public static final Time kEndTriggerDebounce = Seconds.of(0.04);
    public static final PIDConstants kTranslationPID = new PIDConstants(5.5,0,0);
    public static final PIDConstants kRotationPID = new PIDConstants(5.0,0,0);
    public static final PathConstraints kAutoPathConstraints = new PathConstraints(2.0, 2.0, 1/2 * Math.PI, 1 * Math.PI); //? consider making these more aggressive
 
    public static final PPHolonomicDriveController kDriveController = new PPHolonomicDriveController(
      AutoAlign.kTranslationPID, 
      AutoAlign.kRotationPID
  );

    public static final PPHolonomicDriveController kAutoAlignPIDController = new PPHolonomicDriveController(
        AutoAlign.kTranslationPID, 
        AutoAlign.kRotationPID
    );
  }
  public static final class shooter {
    public static final int feedmotor = 22;
    public static final int shootermotor = 13;
    public static final int shootermotor2 = 14;
    public static final int hoodmotor = 24;
    public static final double ForwardLimit = 8.0;
    public static final double ReverseLimit = 0.0;
    public static final double hoodp = 0.1;
    public static final double hoodi = 0.0;
    public static final double hoodd = 0.0;
    public static final double SpeedB = 0;
    public static final double SpeedC = 4200;
    public static final double AngleB = 0;
    public static final double AngleC = 1;
  }
  public static final class climber {
    public static final int climbermotor = 23;
    public static final double forwardlimit = 1.0; 
    public static final double reverselimit = 1.0; 
  }
  public static final class hopper {
    public static final int hoppermotor = 21;
    public static final double forwardlimit = 1.0; 
    public static final double reverselimit = 0; 
  }
  public static final class intake {
    public static final int motor1ID = 12;
    public static final int motor2ID = 13;
  }
  public static final class vision {
    // Constants such as camera and target height stored. Change per robot and goal!
    public static final double CAMERA_HEIGHT_METERS = Units.inchesToMeters(22);

    // Angle between horizontal and the camera.
    public static final double CAMERA_PITCH_RADIANS = Units.degreesToRadians(25);

    // How far from the target we want to be
    public static final double goalRangeMeters = Units.feetToMeters(10);

    public static final double linearP = 2;
    public static final double linearI = 0.0;
    public static final double linearD = 0.05;

    public static final double angularP = 1.84;
    public static final double angularI = 0.08;
    public static final double angularD = 0.1;

    public static final double speakerAimCloseRange = Units.feetToMeters(3.5);
    public static final double speakerAimCloseAngle = -54;
    public static final double speakerAimFarRange = Units.feetToMeters(9);
    public static final double speakerAimFarAngle = -37;

  }
}
