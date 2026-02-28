package frc.robot.commands;
import frc.robot.subsystems.Shooter;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.Intake;

public final class Shoot extends Command {
    Shooter m_shooter;
    double DistanceToHub;
    SwerveSubsystem m_swerve;
    Intake m_intake;
    public Shoot (Shooter m_Shooter, Intake m_intake, SwerveSubsystem m_swerve){
        addRequirements(m_Shooter);
        this.m_shooter=m_Shooter;
        this.m_intake=m_intake;
        this.m_swerve=m_swerve;
        
       
    }
     public void SetMotorSpeed(){
        m_shooter.setShootSpeed(DistanceToHub*Constants.shooter.SpeedB+Constants.shooter.SpeedC);
    }

    public void SetHoodAngle(){
        m_shooter.moveHood(DistanceToHub*Constants.shooter.AngleB+Constants.shooter.AngleC);
    }

    @Override
    public void initialize(){
        DistanceToHub = m_swerve.distancetohub;
        System.out.println("DistancetoHub: " + DistanceToHub);
        SetMotorSpeed();
        SetHoodAngle();
        m_shooter.toggleShooter();
        m_intake.toggleFeed();
    }
}
