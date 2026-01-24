package frc.robot.commands;

import frc.robot.subsystems.SwerveSubsystem;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandScheduler;


public final class AimAtHub extends Command {
    SwerveSubsystem swerve;
    Command toExecute;
    Pose2d currentPose;
    PhotonTrackedTarget target;
    PhotonPipelineResult result;
    Pose2d robotOffset;
    public AimAtHub(SwerveSubsystem swerve){
        addRequirements(swerve);
        this.swerve=swerve;
    }
    
    private boolean isRedAlliance()
    {
        var alliance = DriverStation.getAlliance();
        return alliance.isPresent() ? alliance.get() == DriverStation.Alliance.Red : false;
    }

    @Override
    public void initialize(){
        int targetTag = 26;
        if (isRedAlliance()){
        targetTag = 10;
        }
        Pose2d tagpose = swerve.GetTagPose(targetTag);
        Pose2d robotpose = swerve.getPose();
        //Transform2d transform = tagpose.minus(robotpose);
        //Pose2d relativePose = tagpose.relativeTo(robotpose);
        // Get the rotation (angle) needed
        //Rotation2d angleToTarget = relativePose.getRotation();
        //double degreesToTurn = angleToTarget.getDegrees(); //transform.getRotation().getDegrees();
        double degreesToTurn=0;
        double angle_rad = Math.atan2(
            tagpose.getY() - robotpose.getY(),
            tagpose.getX() - robotpose.getX()
        );
        degreesToTurn = angle_rad*180/Math.PI;
        System.out.println("Turn Degrees "+degreesToTurn);
        robotOffset = new Pose2d(robotpose.getX(),robotpose.getY(),Rotation2d.fromDegrees(robotpose.getRotation().getDegrees()+degreesToTurn));
        toExecute = swerve.driveToPose(robotOffset);
        //toExecute.schedule();
        CommandScheduler.getInstance().schedule(toExecute);
    }
    @Override
    public void execute(){
        var cpose = swerve.getPose();
        System.out.println("X"+cpose.getX()+"Y"+cpose.getY()+"rot"+cpose.getRotation().getDegrees());
    }
    @Override
    public void end(boolean interrupted){

        toExecute.cancel();
    }
    @Override
    public boolean isFinished(){
        //var cpose = swerve.getPose();
        //if ((cpose.getX()-robotOffset.getX()<0.5) && (cpose.getY()-robotOffset.getY()<0.5)){
          //  return true;
        //}
        //return false;
        return toExecute.isFinished();
    }
}