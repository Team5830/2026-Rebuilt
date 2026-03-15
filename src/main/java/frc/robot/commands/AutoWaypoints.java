package frc.robot.commands;

import java.util.List;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.subsystems.SwerveSubsystem;

/**
 * Drives the robot to a target pose using PathPlanner, automatically routing around field obstacles
 * (the central structure) when the robot is on the far side of the field.
 */
public final class AutoWaypoints extends Command {
    private static final double BLUE_WALL_X   = 4.8;
    private static final double RED_WALL_X    = 12.0;
    private static final double FIELD_MID_Y   = 4.0;

    // Blue-side waypoints (pre-defined to avoid repeated allocation)
    private static final Pose2d BLUE_WP_TOP_1  = new Pose2d(5.65, 7.3, Rotation2d.k180deg);
    private static final Pose2d BLUE_WP_TOP_2  = new Pose2d(3.5,  7.3, Rotation2d.k180deg);
    private static final Pose2d BLUE_WP_BOT_1  = new Pose2d(5.72, 0.64, Rotation2d.k180deg);
    private static final Pose2d BLUE_WP_BOT_2  = new Pose2d(3.49, 0.64, Rotation2d.k180deg);

    private final SwerveSubsystem swerve;
    private       Pose2d          targetPose;
    private       Command         pathcommand;

    public AutoWaypoints(SwerveSubsystem swerve, Pose2d targetPose) {
        addRequirements(swerve);
        this.swerve     = swerve;
        this.targetPose = targetPose;
    }

    @Override
    public void initialize() {
        Pose2d currentPose = swerve.getPose();
        boolean red        = swerve.isRedAlliance();

        // Flip target to red side if needed
        Pose2d resolvedTarget = red ? FlippingUtil.flipFieldPose(targetPose) : targetPose;

        // Determine if the robot is on the side of the field that requires routing around the wall
        boolean needsRouting = red
                               ? currentPose.getX() < RED_WALL_X
                               : currentPose.getX() > BLUE_WALL_X;

        PathPlannerPath path;
        if (needsRouting) {
            // Choose top or bottom route based on Y position
            Pose2d wp1, wp2;
            if (currentPose.getY() > FIELD_MID_Y) {
                wp1 = red ? FlippingUtil.flipFieldPose(BLUE_WP_TOP_1) : BLUE_WP_TOP_1;
                wp2 = red ? FlippingUtil.flipFieldPose(BLUE_WP_TOP_2) : BLUE_WP_TOP_2;
            } else {
                wp1 = red ? FlippingUtil.flipFieldPose(BLUE_WP_BOT_1) : BLUE_WP_BOT_1;
                wp2 = red ? FlippingUtil.flipFieldPose(BLUE_WP_BOT_2) : BLUE_WP_BOT_2;
            }
            List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
                new Pose2d(currentPose.getTranslation(),
                           swerve.getPathVelocityHeading(swerve.getFieldVelocity(), wp1)),
                wp1, wp2, resolvedTarget);
            path = buildPath(waypoints, resolvedTarget);
        } else {
            List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
                new Pose2d(currentPose.getTranslation(),
                           swerve.getPathVelocityHeading(swerve.getFieldVelocity(), resolvedTarget)),
                resolvedTarget);
            path = buildPath(waypoints, resolvedTarget);
        }

        pathcommand = swerve.drivePath(path);
        CommandScheduler.getInstance().schedule(pathcommand);
    }

    private PathPlannerPath buildPath(List<Waypoint> waypoints, Pose2d target) {
        PathPlannerPath path = new PathPlannerPath(
            waypoints,
            swerve.constraints,
            new IdealStartingState(swerve.getVelocityMagnitude(swerve.getFieldVelocity()), swerve.getHeading()),
            new GoalEndState(0.0, target.getRotation()));
        path.preventFlipping = true;
        return path;
    }

    @Override
    public void execute() {}

    @Override
    public void end(boolean interrupted) {
        if (pathcommand != null) pathcommand.cancel();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
