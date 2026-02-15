package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.SwerveSubsystem;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;


public final class AimAtHub extends Command {
    SwerveSubsystem swerve;
    CommandXboxController joystick;
    Command aimcmd;
    Pose2d currentPose;
    Pose2d robotOffset;
    double targetAngleDegrees = 0;
    int targetTag = 0;
    public AimAtHub(SwerveSubsystem swerve, CommandXboxController joystick){
        addRequirements(swerve);
        this.swerve=swerve;
        this.joystick = joystick;
    }
    
    private boolean isRedAlliance()
    {
        var alliance = DriverStation.getAlliance();
        return alliance.isPresent() ? alliance.get() == DriverStation.Alliance.Red : false;
    }

    private double angleToTag(){
        Pose2d tagpose = swerve.GetTagPose(targetTag);
        tagpose = new Pose2d(tagpose.getX() +0.6096, tagpose.getY(), tagpose.getRotation()); //0.6096m = 24 in
            if(isRedAlliance()){
                tagpose = new Pose2d(tagpose.getX() -0.6096, tagpose.getY(), tagpose.getRotation()); //0.6096m = 24 in
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
        double targetAngleRad =  angleToTag();
        targetAngleDegrees = targetAngleRad*180/Math.PI;
        System.out.println(" Target heading: "+targetAngleDegrees);
       
        aimcmd = swerve.oneDriveCommand(
        () -> MathUtil.applyDeadband(-joystick.getRawAxis(1), Constants.controller.LEFT_Y_DEADBAND), // X
        () -> MathUtil.applyDeadband(-joystick.getRawAxis(0), Constants.controller.LEFT_X_DEADBAND), // Y 
        () -> Math.sin(angleToTag()),                                                               // Angle 1
        () -> Math.cos(angleToTag())                                                              // Angle 2
        ).onlyWhile(()->Math.hypot(joystick.getRawAxis(4), joystick.getRawAxis(5)) < 0.1);
        CommandScheduler.getInstance().schedule(aimcmd);//.withTimeout(1.0)
        
    }
    @Override
    public void execute(){
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