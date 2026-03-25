package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

/**
 * One-shot command that configures shooter speed and hood angle based on the
 * current distance to hub, then toggles the shooter and feed.
 */
public final class FireOnce extends Command {

    private final Shooter        m_shooter;
    private final Intake         m_intake;
    private final SwerveSubsystem m_swerve;
     private static final double SPEED_TIMEOUT_SECS = 5.0;
    private static final double SHOT_TIMEOUT_SECS  = 3.0;

    public FireOnce(Shooter shooter, Intake intake, SwerveSubsystem swerve) {
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
        if((m_shooter.shooterIsOn)){
            new SequentialCommandGroup(m_intake.FeedOff(), m_shooter.FeedOff(), m_shooter.ShootOff()).schedule();
        }else{
           double speed = distanceToHub * Constants.shooter.SpeedB + Constants.shooter.SpeedC;
            double angle = distanceToHub * Constants.shooter.AngleB + Constants.shooter.AngleC;

            new SequentialCommandGroup(
                 // 1. Spin up wheels
                m_shooter.ShootOn(),
                // 2. Wait until wheels reach target speed (5s safety timeout)
                new WaitUntilCommand(m_shooter::shooterAtTargetSpeed)
                    .withTimeout(SPEED_TIMEOUT_SECS),
                // 3. Turn on feed
                m_shooter.FeedOn(),
                m_intake.FeedOn(),
                // 4. Wait until shot is detected (3s safety timeout)
                new WaitUntilCommand(m_shooter::shotDetected)
                    .withTimeout(SHOT_TIMEOUT_SECS),
                // 5. Turn off feed — ball has cleared
                m_shooter.FeedOff(),
                m_intake.FeedOff()
            ).schedule();
     }
    }

    @Override
    public boolean isFinished() {
        return true; // One-shot: finishes after initialize()
    }
}
