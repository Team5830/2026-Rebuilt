package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.wpilibj2.command.WaitCommand;

/**
 * One-shot command that configures shooter speed and hood angle based on the
 * current distance to hub, then toggles the shooter and feed.
 */
public final class Shoot extends Command {

    private final Shooter        m_shooter;
    private final Intake         m_intake;
    private final SwerveSubsystem m_swerve;

    public Shoot(Shooter shooter, Intake intake, SwerveSubsystem swerve) {
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
        System.out.println("Set Shoot Speed: " + (distanceToHub * Constants.shooter.SpeedB + Constants.shooter.SpeedC));
        System.out.println("moveHood: " + (distanceToHub * Constants.shooter.AngleB + Constants.shooter.AngleC));
        // Configure speed and angle based on range, then toggle shooter + feed
        m_shooter.setShootSpeed(distanceToHub * Constants.shooter.SpeedB + Constants.shooter.SpeedC).schedule();
        m_shooter.moveHood(distanceToHub * Constants.shooter.AngleB + Constants.shooter.AngleC).schedule();
        m_shooter.toggleShooter().schedule();
        new WaitCommand(3.0).andThen( new ParallelCommandGroup(m_shooter.toggleFeed(),m_intake.toggleFeedMode()) ).schedule();
        
    }

    @Override
    public boolean isFinished() {
        return true; // One-shot: finishes after initialize()
    }
}
