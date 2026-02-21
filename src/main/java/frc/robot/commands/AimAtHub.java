package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.SwerveSubsystem;
import frc. robot. subsystems.Lights;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public final class AimAtHub extends Command {
    SwerveSubsystem swerve;
    CommandXboxController joystick;
    Command aimcmd;
    Pose2d currentPose;
    Pose2d robotOffset;
    double targetAngleDegrees = 0;
    int targetTag = 0;
    Lights lights;
    public AimAtHub(SwerveSubsystem swerve, CommandXboxController joystick, Lights lights){
        addRequirements(swerve);
        this.swerve=swerve;
        this.joystick = joystick;
        this.lights=lights;
    }
    
    private boolean isRedAlliance()
    {
        var alliance = DriverStation.getAlliance();
        return alliance.isPresent() ? alliance.get() == DriverStation.Alliance.Red : false;
    }

    private double angleToHub(){
        Pose2d tagpose = swerve.GetTagPose(targetTag);
        //Actually want to aim at hub behind tag so add or subtract 24",
        if (isRedAlliance()){
            tagpose =  new Pose2d(tagpose.getX()-24*2.54/100,tagpose.getY(),tagpose.getRotation());
        }else{
            tagpose =  new Pose2d(tagpose.getX()+24*2.54/100,tagpose.getY(),tagpose.getRotation());
        }
        Pose2d robotpose = swerve.getPose();
        double angle_rad = Math.atan2(
            tagpose.getY() - robotpose.getY(),
            tagpose.getX() - robotpose.getX()
        );
        return angle_rad;
    }

    @Override
    public void initialize(){
        targetTag = 26;
        if (isRedAlliance()){
        targetTag = 10;
        }
        Pose2d tag_pose = swerve.GetTagPose(targetTag);
        System.out.println("Tag: "+ targetTag+ " X: "+tag_pose.getX()+" Y: "+tag_pose.getY()); 
        double targetAngleRad =  angleToHub();
        double distanceToTag = swerve.getDistanceToTag(targetTag);
        targetAngleDegrees = targetAngleRad*180/Math.PI;
        System.out.println(" Target heading: "+targetAngleDegrees + " Distance: " + String.format("%.2f",distanceToTag));
       
        aimcmd = swerve.oneDriveCommand(
        () -> MathUtil.applyDeadband(-joystick.getRawAxis(1), Constants.controller.LEFT_Y_DEADBAND), // X
        () -> MathUtil.applyDeadband(-joystick.getRawAxis(0), Constants.controller.LEFT_X_DEADBAND), // Y 
        () -> Math.sin(angleToHub()),                                                               // Angle 1
        () -> Math.cos(angleToHub())                                                              // Angle 2
        ).onlyWhile(()->Math.hypot(joystick.getRawAxis(4), joystick.getRawAxis(5)) < 0.1);
        CommandScheduler.getInstance().schedule(aimcmd);//.withTimeout(1.0)
        
    }
    @Override
    public void execute(){
        double distanceToTag = swerve.getDistanceToTag(targetTag);
        if (Math.abs(swerve.getPose().getRotation().getDegrees()-targetAngleDegrees) < 2){
            lights.green().schedule();
        }
        
        SmartDashboard.putNumber("DistanceToHub", distanceToTag);
    }
    @Override
    public void end(boolean interrupted){
        System.out.println("Cancelling");
        aimcmd.cancel();
    }
    @Override
    public boolean isFinished(){
        return false;
    }
}
