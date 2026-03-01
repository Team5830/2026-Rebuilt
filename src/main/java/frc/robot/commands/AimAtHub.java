package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.SwerveSubsystem;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/**
 * Aims the robot at the scoring hub for the current alliance.
 *
 * <p>On initialize, computes the hub angle once and schedules a drive command that
 * locks the heading to the hub while the driver can still translate freely.
 * The drive command automatically cancels when the driver pushes the right stick.
 */
public final class AimAtHub extends Command {

    // How far behind the tag the actual hub centre is (metres)
    private static final double HUB_OFFSET_M = Units.inchesToMeters(24);

    // Blue/red hub tag IDs (must match SwerveSubsystem constants)
    private static final int HUB_TAG_BLUE = 26;
    private static final int HUB_TAG_RED  = 10;

    private final SwerveSubsystem        swerve;
    private final CommandXboxController  joystick;
    private final Lights                 lights;

    private int     targetTag;
    private double  targetAngleDegrees;
    private Command aimCmd;

    public AimAtHub(SwerveSubsystem swerve, CommandXboxController joystick, Lights lights) {
        addRequirements(swerve);
        this.swerve   = swerve;
        this.joystick = joystick;
        this.lights   = lights;
    }

    /** Angle (radians) from the robot to the hub centre behind the target tag. */
    private double angleToHub() {
        Pose2d tagPose  = swerve.GetTagPose(targetTag);
        double offsetX  = swerve.isRedAlliance() ? -HUB_OFFSET_M : HUB_OFFSET_M;
        Pose2d hubPose  = new Pose2d(tagPose.getX() + offsetX, tagPose.getY(), tagPose.getRotation());
        Pose2d robotPose = swerve.getPose();
        return Math.atan2(hubPose.getY() - robotPose.getY(), hubPose.getX() - robotPose.getX());
    }

    @Override
    public void initialize() {
        targetTag = swerve.isRedAlliance() ? HUB_TAG_RED : HUB_TAG_BLUE;

        Pose2d tagPose = swerve.GetTagPose(targetTag);
        double targetAngleRad = angleToHub();
        targetAngleDegrees    = Math.toDegrees(targetAngleRad);
        double distanceToTag  = swerve.getDistanceToTag(targetTag);

        System.out.printf("Tag: %d  X: %.2f  Y: %.2f  Heading: %.1f°  Distance: %.2f m%n",
            targetTag, tagPose.getX(), tagPose.getY(), targetAngleDegrees, distanceToTag);

        // Drive command: hold heading on hub, cancel when driver steers
        aimCmd = swerve.oneDriveCommand(
            () -> MathUtil.applyDeadband(-joystick.getRawAxis(1), Constants.controller.LEFT_Y_DEADBAND),
            () -> MathUtil.applyDeadband(-joystick.getRawAxis(0), Constants.controller.LEFT_X_DEADBAND),
            () -> Math.sin(angleToHub()),
            () -> Math.cos(angleToHub())
        ).onlyWhile(() -> Math.hypot(joystick.getRawAxis(4), joystick.getRawAxis(5)) < 0.1);

        aimCmd.initialize();
    }

    @Override
    public void execute() {
        aimCmd.execute();
        double distanceToTag = swerve.getDistanceToTag(targetTag);
        double headingError  = Math.abs(swerve.getPose().getRotation().getDegrees() - targetAngleDegrees);

        if (headingError < 2.0) {
            lights.green().schedule();
        }

        SmartDashboard.putNumber("DistanceToHub", distanceToTag);
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("AimAtHub: cancelling");
        if (aimCmd != null) aimCmd.cancel();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
