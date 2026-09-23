package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveSubsystem;

/**
 * Toggle command: first press spins up the shooter (speed/angle based on
 * distance to hub) and starts feeding once at speed; second press
 * (interruption) shuts the shooter and feed back off.
 */
public final class Shoot extends Command {

    private final Shooter         m_shooter;
    private final Intake          m_intake;
    private final SwerveSubsystem m_swerve;
    private Command shootcmd;

    public Shoot(Shooter shooter, Intake intake, SwerveSubsystem swerve) {
        addRequirements(shooter, intake);
        this.m_shooter = shooter;
        this.m_intake  = intake;
        this.m_swerve  = swerve;
    }

    @Override
    public void initialize() {
        double distanceToHub = m_swerve.DistancetoHub();
        SmartDashboard.putNumber("DistanceToHub", distanceToHub);

        double speed = distanceToHub * Constants.shooter.SpeedB + Constants.shooter.SpeedC;
        double angle = distanceToHub * Constants.shooter.AngleB + Constants.shooter.AngleC;

        System.out.println("DistanceToHub: " + distanceToHub);
        System.out.println("Set Shoot Speed: " + speed);
        System.out.println("moveHood: " + angle);

        shootcmd = new SequentialCommandGroup(
            m_shooter.setShootSpeed(speed),
            m_shooter.moveHood(angle),
            m_shooter.shootOn(),
            new WaitUntilCommand(m_shooter::shooterAtTargetSpeed).withTimeout(5.0),
            m_intake.FeedOn()
        );

        // Drive its lifecycle directly instead of scheduling it, since
        // scheduling would fight this command for the same requirements.
        shootcmd.initialize();
    }

    @Override
    public void execute() {
        if (shootcmd != null && !shootcmd.isFinished()) {
            shootcmd.execute();
        }
    }

    @Override
    public boolean isFinished() {
        // Never ends on its own — stays "on" until the toggle cancels it.
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        if (shootcmd != null) {
            shootcmd.end(true);
        }
        m_shooter.shootOff().schedule();
        m_intake.FeedOff().schedule();
    }
}