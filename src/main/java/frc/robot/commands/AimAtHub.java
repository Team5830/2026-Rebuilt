package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public final class AimAtHub extends Command {
    // Hub is 24 inches (converted to meters) behind the tag
    private static final double HUB_OFFSET_M = 24 * 2.54 / 100.0;
    private static final int    BLUE_TAG      = 26;
    private static final int    RED_TAG       = 10;
    private static final double AIM_TOLERANCE_DEG = 2.0;

    private final SwerveSubsystem    swerve;
    private final edu.wpi.first.wpilibj2.command.button.CommandXboxController joystick;
    private final Lights             lights;

    private int     targetTag;
    private double  targetAngleDegrees;
    private Command aimcmd;

    public AimAtHub(SwerveSubsystem swerve,
                    edu.wpi.first.wpilibj2.command.button.CommandXboxController joystick,
                    Lights lights) {
        addRequirements(swerve);
        this.swerve   = swerve;
        this.joystick = joystick;
        this.lights   = lights;
    }


    /** Returns the angle (radians) from the robot to the hub behind the selected tag. */
    private double angleToHub() {
        boolean red     = swerve.isRedAlliance();
        Pose2d tagpose  = swerve.GetTagPose(targetTag);
        // Offset toward hub behind tag
        double offsetX  = red ? -HUB_OFFSET_M : HUB_OFFSET_M;
        Pose2d hubpose  = new Pose2d(tagpose.getX() + offsetX, tagpose.getY(), tagpose.getRotation());
        Pose2d robot    = swerve.getPose();
        return Math.atan2(hubpose.getY() - robot.getY(), hubpose.getX() - robot.getX());
    }

    @Override
    public void initialize() {
        targetTag = swerve.isRedAlliance() ? RED_TAG : BLUE_TAG;

        //Pose2d tagPose     = swerve.GetTagPose(targetTag);
        //double distanceToTag = swerve.getDistanceToTag(targetTag);
        double targetAngleRad = angleToHub();
        targetAngleDegrees    = Math.toDegrees(targetAngleRad);

        //System.out.printf("AimAtHub: Tag=%d  X=%.2f  Y=%.2f  Heading=%.1f°  Dist=%.2fm%n",
        //                  targetTag, tagPose.getX(), tagPose.getY(),
         //                 targetAngleDegrees, distanceToTag);

        aimcmd = swerve.fieldDriveCommand(
            () -> MathUtil.applyDeadband(-joystick.getRawAxis(1), Constants.controller.LEFT_Y_DEADBAND),
            () -> MathUtil.applyDeadband(-joystick.getRawAxis(0), Constants.controller.LEFT_X_DEADBAND),
            () -> Math.sin(angleToHub()),
            () -> Math.cos(angleToHub())
        ).onlyWhile(() -> Math.hypot(joystick.getRawAxis(4), joystick.getRawAxis(5)) < 0.1);

        aimcmd.initialize();
    }

    @Override
    public void execute() {
        aimcmd.execute();

        double distanceToTag = swerve.getDistanceToTag(targetTag);
        SmartDashboard.putNumber("DistanceToHub", distanceToTag);

        double headingError = Math.abs(swerve.getPose().getRotation().getDegrees() - targetAngleDegrees);
        //System.out.printf("AimAtHub: Tag=%d Heading=%.1f° Error=%.1f° Dist=%.2fm%n",
        //                  targetTag, targetAngleDegrees, headingError, distanceToTag);

        if (headingError < AIM_TOLERANCE_DEG) {
            lights.green_on().schedule();
        }else{
            lights.off().schedule();
        }
    }

    @Override
    public void end(boolean interrupted) {
        //System.out.println("AimAtHub ended (interrupted=" + interrupted + ")");
        if (aimcmd != null) aimcmd.cancel();
        lights.off().schedule();
    }

    @Override
    public boolean isFinished() {
        return aimcmd != null && aimcmd.isFinished();
    }
}
