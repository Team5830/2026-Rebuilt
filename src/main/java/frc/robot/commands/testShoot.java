package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveSubsystem;

/**
 * Test version of the shoot command — reads speed and hood angle from
 * SmartDashboard instead of calculating from distance to hub.
 */
public final class testShoot extends Command {

    private final Shooter         m_shooter;
    private final Intake          m_intake;
    private final SwerveSubsystem m_swerve;

    public testShoot(Shooter shooter, Intake intake, SwerveSubsystem swerve) {
        addRequirements(shooter,intake);
        this.m_shooter = shooter;
        this.m_intake  = intake;
        this.m_swerve  = swerve;
    }

    @Override
    public void initialize() {
        double speed = SmartDashboard.getNumber("ShooterSpeed", 3500);
        double angle = SmartDashboard.getNumber("HoodAngle", 0);

        new SequentialCommandGroup(
            m_shooter.setShootSpeed(speed),
            m_shooter.moveHood(angle),
            m_shooter.shootOn(),
            new WaitUntilCommand(m_shooter::shooterAtTargetSpeed).withTimeout(5.0),
            m_intake.FeedOn(),
            m_shooter.keysToTheKingdomToggle(() -> m_intake.FeedOff().schedule())
        ).schedule();
    }

    @Override
    public boolean isFinished() { return true; }
}
