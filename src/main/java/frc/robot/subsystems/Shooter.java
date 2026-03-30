package frc.robot.subsystems;
 
import java.util.Set;
 
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
 
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
 
public class Shooter extends SubsystemBase {
 
    // -------------------------------------------------------------------------
    // Hardware
    // -------------------------------------------------------------------------
 
    private final SparkMax                  feedMotor;
    private final SparkFlex                 shooterMotor;
    private final SparkFlex                 shooterMotor2;
    private final SparkMax                  hoodMotor;
    private final SparkMax                  St_PetersMotor;
 
    private final SparkClosedLoopController hoodController;
    private final RelativeEncoder           hoodEncoder;
    private final RelativeEncoder           shooterEncoder;
 
    // -------------------------------------------------------------------------
    // State
    // -------------------------------------------------------------------------
 
    private double  shootSpeed       = 3500;
    public  boolean shooterIsOn      = false;
    private boolean feedIsOn         = false;
    private boolean intakeFeedIsOn   = false;
    private boolean gateOpen         = false;
    private boolean gateReject       = false;
 
    // -------------------------------------------------------------------------
    // Filters
    // -------------------------------------------------------------------------
 
    private final MedianFilter  aveFilt              = new MedianFilter(50);
    private final LinearFilter  shooterBaselineFilter = LinearFilter.movingAverage(75); // ~1.5s window
    private final MedianFilter  shooterCurrentFilt   = new MedianFilter(5);
    private final MedianFilter  feederCurrentFilt    = new MedianFilter(5);
 
    private boolean lastShotDetected = false;
 
    // -------------------------------------------------------------------------
    // Constructor
    // -------------------------------------------------------------------------
 
    @SuppressWarnings("removal")
    public Shooter() {
        SparkMax  feed    = null;
        SparkMax  hood    = null;
        SparkMax  stPeter = null;
        SparkFlex shoot1  = null;
        SparkFlex shoot2  = null;
 
        try {
            feed    = new SparkMax( Constants.shooter.feedMotor,      MotorType.kBrushless);
            shoot1  = new SparkFlex(Constants.shooter.shooterMotor,   MotorType.kBrushless);
            shoot2  = new SparkFlex(Constants.shooter.shooterMotor2,  MotorType.kBrushless);
            hood    = new SparkMax( Constants.shooter.hoodMotor,      MotorType.kBrushless);
            stPeter = new SparkMax( Constants.shooter.St_PetersMotor,  MotorType.kBrushless);
        } catch (RuntimeException ex) {
            DriverStation.reportError("Error instantiating Shooter: " + ex.getMessage(), true);
        }
 
        feedMotor     = feed;
        shooterMotor  = shoot1;
        shooterMotor2 = shoot2;
        hoodMotor     = hood;
        St_PetersMotor = stPeter;
 
        // --- Shooter wheels ---
        SparkFlexConfig shooterConfig = new SparkFlexConfig();
        shooterConfig.idleMode(IdleMode.kCoast);
        shooterConfig.smartCurrentLimit(30);
        shooterConfig.closedLoopRampRate(0.3);
        shooterConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(0.003, 0.0, 0.18)
            .feedForward.kV(0.0002);
        configureMotorFlex(shooterMotor, shooterConfig, "shooter");
        shooterEncoder = (shooterMotor != null) ? shooterMotor.getEncoder() : null;
 
        // --- Feed roller ---
        SparkMaxConfig feedConfig = new SparkMaxConfig();
        feedConfig.idleMode(IdleMode.kCoast);
        configureMotor(feedMotor, feedConfig, "feed");
 
        // --- St. Peter's gate motor ---
        SparkMaxConfig stPeterConfig = new SparkMaxConfig();
        stPeterConfig.idleMode(IdleMode.kCoast);
        configureMotor(St_PetersMotor, stPeterConfig, "St Peter");
 
        // --- Shooter follower ---
        SparkFlexConfig shooter2Config = new SparkFlexConfig();
        shooter2Config.idleMode(IdleMode.kCoast);
        shooter2Config.smartCurrentLimit(30);
        shooter2Config.closedLoopRampRate(0.3);
        if (shooterMotor != null) shooter2Config.follow(shooterMotor, true);
        configureMotorFlex(shooterMotor2, shooter2Config, "shooter2");
 
        // --- Hood ---
        SparkMaxConfig hoodConfig = new SparkMaxConfig();
        hoodConfig.encoder.positionConversionFactor(Constants.shooter.multiplier);
        hoodConfig.idleMode(IdleMode.kBrake);
        hoodConfig.softLimit
            .forwardSoftLimit(Constants.shooter.ForwardLimit)
            .forwardSoftLimitEnabled(true)
            .reverseSoftLimit(Constants.shooter.ReverseLimit)
            .reverseSoftLimitEnabled(true);
        hoodConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .p(Constants.shooter.hoodp)
            .i(Constants.shooter.hoodi)
            .d(Constants.shooter.hoodd)
            .outputRange(-1, 1);
        hoodConfig.smartCurrentLimit(30);
        configureMotor(hoodMotor, hoodConfig, "hood");
 
        if (hoodMotor != null) {
            hoodController = hoodMotor.getClosedLoopController();
            hoodEncoder    = hoodMotor.getEncoder();
        } else {
            hoodController = null;
            hoodEncoder    = null;
        }
    }
 
    // -------------------------------------------------------------------------
    // Motor configuration helpers
    // -------------------------------------------------------------------------
 
    private void configureMotor(SparkMax motor, SparkMaxConfig config, String name) {
        if (motor == null) return;
        REVLibError err = motor.configure(config, ResetMode.kResetSafeParameters,
                                          PersistMode.kPersistParameters);
        if (err != REVLibError.kOk)
            DriverStation.reportError("Failed to configure " + name + " motor: " + err, true);
    }
 
    private void configureMotorFlex(SparkFlex motor, SparkFlexConfig config, String name) {
        if (motor == null) return;
        REVLibError err = motor.configure(config, ResetMode.kResetSafeParameters,
                                          PersistMode.kPersistParameters);
        if (err != REVLibError.kOk)
            DriverStation.reportError("Failed to configure " + name + " motor: " + err, true);
    }
 
    // -------------------------------------------------------------------------
    // Private helpers
    // -------------------------------------------------------------------------
 
    /** Push the current shootSpeed to the closed-loop controller. */
    private void applyShooterSetpoint() {
        if (shooterMotor != null)
            shooterMotor.getClosedLoopController()
                        .setSetpoint(-shootSpeed, ControlType.kVelocity);
    }
 
    // -------------------------------------------------------------------------
    // Sensors / state queries
    // -------------------------------------------------------------------------
 
    public double getShooterSpeed() {
        return shooterEncoder.getVelocity();
    }
 
    public boolean shooterAtTargetSpeed() {
        return Math.abs(shooterEncoder.getVelocity()) >= Math.abs(shootSpeed * 0.9);
    }
 
    /** Returns true on the rising edge of a current spike — indicates a note was fired. */
    public boolean shotDetected() {
        if (shooterMotor == null || !shooterIsOn) return false;
 
        double  current      = shooterMotor.getOutputCurrent();
        double  baseline     = shooterBaselineFilter.calculate(current);
        boolean spike        = current > 5 && current > baseline * 1.2;
        boolean risingEdge   = spike && !lastShotDetected;
        lastShotDetected     = spike;
        return risingEdge;
    }
 
    /** Returns true when feeder current spikes above its rolling average — indicates a note is present. */
    public boolean monitorFeed() {
        double current    = feedMotor.getOutputCurrent();
        double average    = feederCurrentFilt.calculate(current);
        return current > average * 1.10;
    }
 
    // -------------------------------------------------------------------------
    // Hood commands
    // -------------------------------------------------------------------------
 
    public Command adjustHoodUp() {
        return runOnce(() -> {
            if (hoodController != null)
                hoodController.setSetpoint(hoodController.getSetpoint() + 1, ControlType.kPosition);
        });
    }
 
    public Command adjustHoodDown() {
        return runOnce(() -> {
            if (hoodController != null)
                hoodController.setSetpoint(hoodController.getSetpoint() - 1, ControlType.kPosition);
        });
    }
 
    public Command moveHood(double position) {
        return runOnce(() -> {
            if (hoodController != null)
                hoodController.setSetpoint(position, ControlType.kPosition);
        });
    }
 
    // -------------------------------------------------------------------------
    // Shooter commands
    // -------------------------------------------------------------------------
 
    /**
     * Update shoot speed. If the shooter is already running, applies the new
     * setpoint immediately so the wheels ramp without needing a toggle.
     */
    public Command setShootSpeed(double setpoint) {
        return runOnce(() -> {
            shootSpeed = setpoint;
            if (shooterIsOn) applyShooterSetpoint();
        });
    }
 
    public Command shootOn() {
        return runOnce(() -> {
            applyShooterSetpoint();
            shooterIsOn = true;
        });
    }
 
    public Command shootOff() {
        return runOnce(() -> {
            if (shooterMotor != null) shooterMotor.setVoltage(0.0);
            shooterIsOn = false;
        });
    }
 
    /** Cut feed, wait briefly, then spin down wheels. */
    public Command shooterOff() {
        shooterIsOn = false;
        return feedOff()
            .andThen(new WaitCommand(0.2))
            .andThen(shootOff());
    }
 
    /** Toggle shooter on/off. */
    public Command toggleShooter() {
        return runOnce(() -> {
            if (shooterIsOn) {
                if (shooterMotor != null) shooterMotor.setVoltage(0.0);
                moveHood(0).schedule();
                shooterIsOn = false;
            } else {
                applyShooterSetpoint();
                shooterIsOn = true;
            }
        });
    }
 
    // -------------------------------------------------------------------------
    // Feed commands
    // -------------------------------------------------------------------------
 
    /** Push fuel to the shooter. Clears intake-feed state. */
    public Command feedOn() {
        return runOnce(() -> {
            if (feedMotor != null) feedMotor.setVoltage(-8);
            feedIsOn       = true;
            intakeFeedIsOn = false;
        });
    }
 
    public Command feedOff() {
        return runOnce(() -> {
            if (feedMotor != null) feedMotor.setVoltage(0.0);
            feedIsOn = false;
        });
    }
 
    public Command toggleFeed() {
        return runOnce(() -> {
            if (feedIsOn) {
                if (feedMotor != null) feedMotor.setVoltage(0.0);
                feedIsOn = false;
            } else {
                if (feedMotor != null) feedMotor.setVoltage(-7);
                feedIsOn       = true;
                intakeFeedIsOn = false;
            }
        });
    }
 
    /** Run feed motor in reverse to assist intaking. Clears shooter-feed state. */
    public Command intakeFeed() {
        return runOnce(() -> {
            if (feedMotor != null) feedMotor.setVoltage(1.5);
            intakeFeedIsOn = true;
            feedIsOn       = false;
        });
    }
 
    public Command intakeFeedOff() {
        return runOnce(() -> {
            if (feedMotor != null) feedMotor.setVoltage(0.0);
            intakeFeedIsOn = false;
        });
    }
 
    /** Toggle between intake-feed and off. */
    public Command toggleIntakeFeed() {
        return runOnce(() -> {
            if (intakeFeedIsOn) {
                if (feedMotor != null) feedMotor.setVoltage(0);
                intakeFeedIsOn = false;
            } else {
                if (feedMotor != null) feedMotor.setVoltage(3);
                intakeFeedIsOn = true;
                feedIsOn       = false;
            }
        });
    }
 
    /** Feed a note until a jam/note-present spike is detected, then stop. */
    public Command indexFeed() {
        return intakeFeed()
            .onlyWhile(() -> !monitorFeed() && !shotDetected())
            .andThen(intakeFeedOff());
    }
 
    // -------------------------------------------------------------------------
    // Gate (St. Peter's Motor) commands
    // -------------------------------------------------------------------------
 
    public Command gateOpen() {
        return runOnce(() -> {
            if (St_PetersMotor != null) St_PetersMotor.setVoltage(8);
            gateOpen   = true;
            gateReject = false;
        });
    }
 
    public Command gateClosed() {
        return runOnce(() -> {
            if (St_PetersMotor != null) St_PetersMotor.setVoltage(0.0);
            gateOpen   = false;
            gateReject = false;
        });
    }
 
    public Command gateRejectToggle() {
        return runOnce(() -> {
            if (gateReject) {
                if (St_PetersMotor != null) St_PetersMotor.setVoltage(0);
                gateReject = false;
                gateOpen = false;
            } else {
                if (St_PetersMotor != null) St_PetersMotor.setVoltage(-4);
                gateOpen   = false;
                gateReject = true;
            }
        });
    }

    public Command GateRejectOn() {
        return runOnce(() -> {
            if (St_PetersMotor != null) {
                St_PetersMotor.setVoltage(-4.0);
            }
            gateOpen = false;
            gateReject = true;
        });
    }

    public Command GateStop() {
        return runOnce(() -> {
            if (St_PetersMotor != null) {
                St_PetersMotor.setVoltage(0.0);
            }
            gateOpen = false;
            gateReject = false;
        });
    }
 
    // -------------------------------------------------------------------------
    // Composite shoot sequence
    // -------------------------------------------------------------------------
 
    /**
     * Toggle the full shoot sequence on/off.
     *
     * ON  path: waits for shooter to reach speed, then feeds and opens the gate.
     * OFF path: cuts feed, closes gate, spins down shooter, and homes the hood.
     *
     * Commands.defer() is used so that both branches and the shooterIsOn condition
     * are evaluated at schedule time, not at command-object construction time.
     */
   public Command keysToTheKingdomToggle() {
    return Commands.defer(
        () -> Commands.either(
            // --- ON path: shooter is already running, fire the note ---
            new SequentialCommandGroup(
                new WaitUntilCommand(this::shooterAtTargetSpeed).withTimeout(5),
                feedOn(),
                gateOpen(),
                new WaitCommand(1),
                gateClosed()
                //new WaitCommand(1)
            ).repeatedly().finallyDo(() -> {
                feedOff().schedule();
                gateClosed().schedule();
                shootOff().schedule();
                moveHood(0).schedule();
            }),
            // --- OFF path: shut everything down ---
            new SequentialCommandGroup(
                feedOff(),
                gateClosed(),
                shootOff(),
                moveHood(0)
            ),
            () -> shooterIsOn
        ),
        Set.of(this)
        );
    }
    // -------------------------------------------------------------------------
    // Telemetry
    // -------------------------------------------------------------------------
 
    @Override
    public void periodic() {
        if (hoodEncoder    != null) SmartDashboard.putNumber("Hood",             hoodEncoder.getPosition());
        if (shooterEncoder != null) SmartDashboard.putNumber("Shooter Velocity", shooterEncoder.getVelocity());
 
        SmartDashboard.putNumber ("Shooter Voltage",        aveFilt.calculate(shooterMotor.getAppliedOutput() * shooterMotor.getBusVoltage()));
        SmartDashboard.putNumber ("Shooter1 Temp",          shooterMotor.getMotorTemperature());
        SmartDashboard.putNumber ("Shooter2 Temp",          shooterMotor2.getMotorTemperature());
        SmartDashboard.putNumber ("Shooter Current",        shooterMotor.getOutputCurrent());
        SmartDashboard.putNumber ("Feeder Current",         feedMotor.getOutputCurrent());
 
        SmartDashboard.putBoolean("Shooter Is On",          shooterIsOn);
        SmartDashboard.putBoolean("Shooter Feed Is On",     feedIsOn);
        SmartDashboard.putBoolean("Shooter IntakeFeed On",  intakeFeedIsOn);
        SmartDashboard.putBoolean("Shot Detected",          shotDetected());
        SmartDashboard.putBoolean("Feed Fuel",              monitorFeed());
    }
}