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
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

/**
 * One-shot command that configures shooter speed and hood angle based on the
 * current distance to hub, then toggles the shooter and feed.
 */
public final class Shoot extends Command {

    private final Shooter        m_shooter;
    private final Intake         m_intake;
    private final SwerveSubsystem m_swerve;
    private Command shootcmd;

    public Shoot(Shooter shooter, Intake intake, SwerveSubsystem swerve) {
        addRequirements(shooter,intake);
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
        
        double speed = distanceToHub * Constants.shooter.SpeedB + Constants.shooter.SpeedC;
        double angle = distanceToHub * Constants.shooter.AngleB + Constants.shooter.AngleC;

        shootcmd = new SequentialCommandGroup(
        m_shooter.setShootSpeed(speed),
        m_shooter.moveHood(angle),
        m_shooter.shootOn(),
        new WaitUntilCommand(m_shooter::shooterAtTargetSpeed).withTimeout(5.0),
        m_intake.FeedOn(),
        m_shooter.keysToTheKingdomToggle(() -> m_intake.FeedOff().schedule())
    );

    shootcmd.schedule();
    }

   @Override
    public boolean isFinished() { return true; }
}
