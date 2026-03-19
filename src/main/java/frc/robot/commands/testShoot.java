package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Constants.shooter;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveSubsystem;

/**
 * One-shot command that configures shooter speed and hood angle based on the
 * current distance to hub, then toggles the shooter and feed.
 */
public final class testShoot extends Command {

    private final Shooter        m_shooter;
    private final Intake         m_intake;
    private final SwerveSubsystem m_swerve;
    private double shooterspeed, hoodangle;

    public testShoot(Shooter shooter, Intake intake, SwerveSubsystem swerve) {
        addRequirements(shooter);
        this.m_shooter = shooter;
        this.m_intake  = intake;
        this.m_swerve  = swerve;
    }

    @Override
    public void initialize() {
        double distanceToHub = m_swerve.DistancetoHub();
        System.out.println("DistanceToHub: " + distanceToHub);
        SmartDashboard.putNumber("DistanceToHub", distanceToHub);
        shooterspeed = SmartDashboard.getNumber("ShooterSpeed", 3500);
        hoodangle = SmartDashboard.getNumber("HoodAngle", 0);
        System.out.println("ShootSpeed: " + shooterspeed);
        System.out.println("HoodAngle: " + hoodangle);
        // Configure speed and angle based on range, then toggle shooter + feed
        
        m_shooter.setShootSpeed(shooterspeed).schedule();
        m_shooter.moveHood(hoodangle).schedule();
        m_shooter.toggleShooter().schedule();
        new WaitCommand(2.0).andThen(m_shooter.toggleFeed()).andThen(m_intake.toggleFeedMode()).schedule();
        //m_shooter.toggleFeed().schedule();
        //m_intake.toggleFeedMode().schedule();
        /* 
        new ParallelCommandGroup(m_shooter.setShootSpeed(shooterspeed), 
            m_shooter.moveHood(hoodangle), 
            m_shooter.toggleShooter()
            ).andThen(
                new WaitCommand(3.0)
            ).andThen( new ParallelCommandGroup(   
                 m_shooter.toggleFeed(),  
                 m_intake.toggleFeedMode() )
            ).schedule();*/
    }

    @Override
    public boolean isFinished() {
        return true; // One-shot: finishes after initialize()
    }
}
