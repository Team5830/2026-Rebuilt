package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.SwerveSubsystem;
import swervelib.SwerveInputStream;
import edu.wpi.first.wpilibj2.command.Command;

import com.ctre.phoenix6.swerve.SwerveModuleConstants.DriveMotorArrangement;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import  edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public final class Aim extends Command {
    // Hub is 24 inches (converted to meters) behind the tag
    private static final double HUB_OFFSET_M = 24 * 2.54 / 100.0;
    private static final int    BLUE_TAG      = 26;
    private static final int    RED_TAG       = 10;
    private static final double AIM_TOLERANCE_DEG = 2.0;

    private SwerveSubsystem          swerve;
    private final SwerveInputStream driveInput;
    private final Lights             lights;

    private int     targetTag;
    private double  targetAngleDegrees;
    private Command aimcmd;
    private CommandXboxController joystick; 

    

    public Aim(SwerveSubsystem swerve, SwerveInputStream driveAngle,
            Lights lights, CommandXboxController joystick ) {
        addRequirements(swerve);
        this.swerve = swerve;
        this.driveInput = driveAngle;
        this.lights   = lights;      
        this.joystick = joystick;          
    }

    @Override
    public void initialize(){
        targetTag = swerve.getAlliance() == DriverStation.Alliance.Red ? RED_TAG : BLUE_TAG;
        Pose2d tagPose     = swerve.GetTagPose(targetTag);

        boolean red = swerve.getAlliance() == DriverStation.Alliance.Red;
        double offsetX  = red ? -HUB_OFFSET_M : HUB_OFFSET_M;
        Pose2d hubpose  = new Pose2d(tagPose.getX() + offsetX, tagPose.getY(), tagPose.getRotation());
        aimcmd = swerve.driveFieldOriented(driveInput.aim(hubpose).headingWhile(true).aimWhile(()->Math.hypot(joystick.getRawAxis(4), joystick.getRawAxis(5)) < 0.1));
        aimcmd.initialize();
    }

    @Override
    public void execute() {
        aimcmd.execute();
        double headingError = Math.abs(swerve.getPose().getRotation().getDegrees() - targetAngleDegrees);
        double distanceToTag = swerve.getDistanceToTag(targetTag);
        System.out.printf("AimAtHub: Tag=%d Heading=%.1f° Error=%.1f° Dist=%.2fm%n",
                          targetTag, targetAngleDegrees, headingError, distanceToTag);

        if (headingError < AIM_TOLERANCE_DEG) {
            lights.green_on().schedule();
        }else{
            lights.red().schedule();
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