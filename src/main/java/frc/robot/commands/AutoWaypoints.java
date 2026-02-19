package frc.robot.commands;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.subsystems.SwerveSubsystem;

public final class AutoWaypoints extends Command{
    SwerveSubsystem swerve;
    Pose2d currentPose;
    Pose2d targetPose;
    Command pathcommand;
    public AutoWaypoints(SwerveSubsystem swerve, Pose2d targetPose){
        addRequirements((swerve));
        this.swerve=swerve;
        this.targetPose=targetPose;
    }
   
    private boolean isRedAlliance()
    {
        var alliance = DriverStation.getAlliance();
        return alliance.isPresent() ? alliance.get() == DriverStation.Alliance.Red : false;
    }
    private Pose2d convertBluetoRed(Pose2d BluePose){
        return (FlippingUtil.flipFieldPose(BluePose));
    }
@Override
    public void initialize(){
        currentPose = swerve.getPose();
        Pose2d WayPoint1,WayPoint2; 
        if (isRedAlliance()){
            targetPose = convertBluetoRed(targetPose);
        }
        if (isRedAlliance()){
             if (currentPose.getX() < 12){       
                                               //If past wall
                if (currentPose.getY() > 4){                                    //If above midpoint
                    WayPoint1 = convertBluetoRed(new Pose2d(5.65, 7.3, Rotation2d.k180deg));
                    WayPoint2 = convertBluetoRed(new Pose2d(3.5, 7.3, Rotation2d.k180deg));
                }
                else{
                    WayPoint1 = convertBluetoRed(new Pose2d(5.72, 0.64, Rotation2d.k180deg));
                    WayPoint2 = convertBluetoRed(new Pose2d(3.49, 0.64, Rotation2d.k180deg));
                }
                List waypoints = PathPlannerPath.waypointsFromPoses(
                new Pose2d(currentPose.getTranslation(), swerve.getPathVelocityHeading(swerve.getFieldVelocity(), WayPoint1)),
                WayPoint1,
                WayPoint2,
                targetPose);
                PathPlannerPath path = new PathPlannerPath(
                    waypoints, swerve.constraints,
                    new IdealStartingState(swerve.getVelocityMagnitude(swerve.getFieldVelocity()), swerve.getHeading()),
                    new GoalEndState(0.0, targetPose.getRotation())
                    );
                    path.preventFlipping = true;
                    pathcommand = swerve.drivePath(path);
                    CommandScheduler.getInstance().schedule(pathcommand);
        
            }else{
                List waypoints = PathPlannerPath.waypointsFromPoses(
                new Pose2d(currentPose.getTranslation(), swerve.getPathVelocityHeading(swerve.getFieldVelocity(), targetPose)),
                    targetPose);
                PathPlannerPath path = new PathPlannerPath(
                    waypoints, swerve.constraints,
                    new IdealStartingState(swerve.getVelocityMagnitude(swerve.getFieldVelocity()), swerve.getHeading()),
                    new GoalEndState(0.0, targetPose.getRotation())
                    );
                    path.preventFlipping = true;
                    pathcommand = swerve.drivePath(path);
                    CommandScheduler.getInstance().schedule(pathcommand);
            }
        }
        else{    
            if (currentPose.getX() > 4.8){       
                                               //If past wall
                if (currentPose.getY() > 4){                                    //If above midpoint
                    WayPoint1 = new Pose2d(5.65, 7.3, Rotation2d.k180deg);
                    WayPoint2 = new Pose2d(3.5, 7.3, Rotation2d.k180deg);
                }
                else{
                    WayPoint1 = new Pose2d(5.72, 0.64, Rotation2d.k180deg);
                    WayPoint2 = new Pose2d(3.49, 0.64, Rotation2d.k180deg);
                }
                List waypoints = PathPlannerPath.waypointsFromPoses(
                new Pose2d(currentPose.getTranslation(), swerve.getPathVelocityHeading(swerve.getFieldVelocity(), WayPoint1)),
                WayPoint1,
                WayPoint2,
                targetPose);
                PathPlannerPath path = new PathPlannerPath(
                    waypoints, swerve.constraints,
                    new IdealStartingState(swerve.getVelocityMagnitude(swerve.getFieldVelocity()), swerve.getHeading()),
                    new GoalEndState(0.0, targetPose.getRotation())
                    );
                    path.preventFlipping = true;
                    pathcommand = swerve.drivePath(path);
                    CommandScheduler.getInstance().schedule(pathcommand);
        
            }else{
                List waypoints = PathPlannerPath.waypointsFromPoses(
                new Pose2d(currentPose.getTranslation(), swerve.getPathVelocityHeading(swerve.getFieldVelocity(), targetPose)),
                    targetPose);
                PathPlannerPath path = new PathPlannerPath(
                    waypoints, swerve.constraints,
                    new IdealStartingState(swerve.getVelocityMagnitude(swerve.getFieldVelocity()), swerve.getHeading()),
                    new GoalEndState(0.0, targetPose.getRotation())
                    );
                    path.preventFlipping = true;
                    pathcommand = swerve.drivePath(path);
                    CommandScheduler.getInstance().schedule(pathcommand);
            }
            }  
    
        }
            //getPathFromWaypoint(new Pose2d(3.235,7.186,Rotation2d.fromDegrees(-78.024))));
           //getPathFromWaypoint(new Pose2d(2.847,4.019,Rotation2d.fromDegrees(0))));
            //getPathFromWaypoint(new Pose2d(1.804,3.965,Rotation2d.fromDegrees(0))));
            //getPathFromWaypoint(new Pose2d(2.901,0.963,Rotation2d.fromDegrees(47.545))));  
            
            
    
    @Override
    public void execute(){
    
        
    }
    @Override
    public void end(boolean interrupted){
        pathcommand.cancel();
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}