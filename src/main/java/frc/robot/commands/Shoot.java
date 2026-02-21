package frc.robot.commands;
import frc.robot.subsystems.Shooter;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;

public final class Shoot extends Command {
    Shooter m_shooter;
    double DistanceToHub;
    SwerveSubsystem m_swerve;
    public Shoot (Shooter m_Shooter){
        addRequirements(m_Shooter);
        this.m_shooter=m_Shooter;
        SmartDashboard.getNumber("DistanceToHub", 2);
       
    }
     public void SetMotorSpeed(){
        m_shooter.setShootSpeed(DistanceToHub*Constants.shooter.SpeedB+Constants.shooter.SpeedC);
    }

    public void SetHoodAngle(){
        m_shooter.moveHood(DistanceToHub*Constants.shooter.AngleB+Constants.shooter.AngleC);
    }

    @Override
    public void initialize(){
        SetMotorSpeed();
        SetHoodAngle();
        m_shooter.toggleShooter();
    }
}
