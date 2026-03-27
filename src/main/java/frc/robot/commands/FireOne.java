package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.Shooter;

/**
 * Spins up the shooter to the current set speed, waits until the wheels reach
 * that speed, turns on the feed, and then turns the feed off as soon as a ball
 * is detected passing through the shooter wheels.
 *
 * The shooter wheels are left running after the command ends so the robot is
 * ready to fire again immediately.
 */
public class FireOne extends SequentialCommandGroup {

    private static final double SPEED_TIMEOUT_SECS = 5.0;
    private static final double SHOT_TIMEOUT_SECS  = 3.0;

    public FireOne(Shooter shooter) {
        addCommands(
            // 1. Spin up wheels
            shooter.shootOn(),

            // 2. Wait until wheels reach target speed (5s safety timeout)
            new WaitUntilCommand(shooter::shooterAtTargetSpeed)
                .withTimeout(SPEED_TIMEOUT_SECS),

            // 3. Turn on feed
            shooter.feedOn(),

            // 4. Wait until shot is detected (3s safety timeout)
            new WaitUntilCommand(shooter::shotDetected)
                .withTimeout(SHOT_TIMEOUT_SECS),

            // 5. Turn off feed — ball has cleared
            shooter.feedOff()
        );
    }
}
