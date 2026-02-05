package frc.robot.commands;

import frc.robot.subsystems.SwerveSubsystem;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;


public final class DriveToShoot extends Command {
    SwerveSubsystem swerve;
    Command toExecute;
    Pose2d currentPose;
    PhotonTrackedTarget target;
    PhotonPipelineResult result;
    Pose2d robotOffset;
    int targetTag;

    public DriveToShoot(SwerveSubsystem swerve){
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
        targetTag = 26;  // 157.79, 158.32 44.25 

        if (isRedAlliance()){
            targetTag = 10; //492.33, 158.32, 44.25 
        }
        currentPose = swerve.getPose();
        Pose2d tagpose = swerve.GetTagPose(targetTag);
        //robotOffset = new Pose2d(0,0,Rotation2d.fromDegrees(0));
        robotOffset =  new Pose2d(tagpose.getX()-.18,tagpose.getY()+.055,tagpose.getRotation().minus(Rotation2d.fromDegrees(180)));
        
        System.out.println("X"+robotOffset.getX()+"Y"+robotOffset.getY()+"rot"+robotOffset.getRotation().getDegrees());
        toExecute = swerve.driveToPoseRobotRelative(robotOffset);
        toExecute.schedule();
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
        var cpose = swerve.getPose();
        //if ((cpose.getX()-robotOffset.getX()<0.5) && (cpose.getY()-robotOffset.getY()<0.5)){
          //  return true;
        //}
        //return false;
        return toExecute.isFinished();
    }
}