package frc.robot.subsystems;

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

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {

    private final SparkMax                  feedmotor;
    private final SparkFlex                 shootermotor;
    private final SparkFlex                 shootermotor2;
    private final SparkMax                  hoodmotor;
    private final SparkClosedLoopController hoodController;
    private final RelativeEncoder           hoodEncoder;
    private final RelativeEncoder           shooterEncoder;

    private double  shootspeed       = 3500;
    public boolean shooterIsOn      = false;
    private boolean feedIsOn         = false;
    private boolean intakeFeedIsOn   = false;

    @SuppressWarnings("removal")
    public Shooter() {
        SparkMax  feed   = null, hood   = null;
        SparkFlex shoot1 = null, shoot2 = null;
        try {
            feed   = new SparkMax( Constants.shooter.feedmotor,     MotorType.kBrushless);
            shoot1 = new SparkFlex(Constants.shooter.shootermotor,  MotorType.kBrushless);
            shoot2 = new SparkFlex(Constants.shooter.shootermotor2, MotorType.kBrushless);
            hood   = new SparkMax( Constants.shooter.hoodmotor,     MotorType.kBrushless);
        } catch (RuntimeException ex) {
            DriverStation.reportError("Error instantiating Shooter: " + ex.getMessage(), true);
        }
        feedmotor     = feed;
        shootermotor  = shoot1;
        shootermotor2 = shoot2;
        hoodmotor     = hood;

        // --- Shooter wheels ---
        SparkFlexConfig shooterConfig = new SparkFlexConfig();
        shooterConfig.idleMode(IdleMode.kCoast);
        shooterConfig.smartCurrentLimit(30);
        shooterConfig.closedLoopRampRate(0.3);
        shooterConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(0.003, 0.0, 0.18);
        configureMotorFlex(shootermotor, shooterConfig, "shooter");
        shooterEncoder = (shootermotor != null) ? shootermotor.getEncoder() : null;

        // --- Feed roller ---
        SparkMaxConfig feedConfig = new SparkMaxConfig();
        feedConfig.idleMode(IdleMode.kCoast);
        configureMotor(feedmotor, feedConfig, "feed");

        // --- Shooter follower ---
        SparkFlexConfig shooter2Config = new SparkFlexConfig();
        shooter2Config.idleMode(IdleMode.kCoast);
        shooter2Config.smartCurrentLimit(30);
        shooter2Config.closedLoopRampRate(0.3);
        if (shootermotor != null) shooter2Config.follow(shootermotor, true);
        configureMotorFlex(shootermotor2, shooter2Config, "shooter2");

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
        configureMotor(hoodmotor, hoodConfig, "hood");

        if (hoodmotor != null) {
            hoodController = hoodmotor.getClosedLoopController();
            hoodEncoder    = hoodmotor.getEncoder();
            //hoodEncoder.setPosition(0);
        } else {
            hoodController = null;
            hoodEncoder    = null;
        }
    }

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

    /** Push the current shootspeed to the closed-loop controller. */
    private void applyShooterSetpoint() {
        if (shootermotor != null)
            shootermotor.getClosedLoopController()
                        .setSetpoint(-shootspeed, ControlType.kVelocity);
    }

    // -------------------------------------------------------------------------
    // Commands
    // -------------------------------------------------------------------------

    public Command adjustHoodup(){
        return runOnce(() -> {
            if (hoodController != null)
                hoodController.setSetpoint(hoodController.getSetpoint()+1, ControlType.kPosition);
        });
    }

    public Command adjustHooddown(){
        return runOnce(() -> {
            if (hoodController != null)
                hoodController.setSetpoint(hoodController.getSetpoint()-1, ControlType.kPosition);
        });
    }

    public Command moveHood(double position) {
        return runOnce(() -> {
            if (hoodController != null)
                hoodController.setSetpoint(position, ControlType.kPosition);
        });
    }

    /** Push fuel to the shooter. Clears intake-feed state. */
    public Command FeedOn() {
        return runOnce(() -> {
            if (feedmotor != null) feedmotor.setVoltage(-6); 
            feedIsOn       = true;
            intakeFeedIsOn = false;
        });
    }

    public Command FeedOff() {
        return runOnce(() -> {
            if (feedmotor != null) feedmotor.setVoltage(0.0);
            feedIsOn = false;
        });
    }

    /** Run feed motor in reverse to assist intaking. Clears shooter-feed state. */
    public Command IntakeFeed() {
        return runOnce(() -> {
            if (feedmotor != null) feedmotor.setVoltage(1.5);
            intakeFeedIsOn = true;
            feedIsOn       = false;
        });
    }

    public Command IntakeFeedOff() {
        return runOnce(() -> {
            if (feedmotor != null) feedmotor.setVoltage(0.0);
            intakeFeedIsOn = false;
        });
    }

    /**
     * Update shoot speed. If the shooter is already running, applies the new
     * setpoint immediately so the wheels ramp without needing a toggle.
     */
    public Command setShootSpeed(double setpoint) {
        return runOnce(() -> {          // fix: was branching outside lambda
            shootspeed = setpoint;
            if (shooterIsOn) applyShooterSetpoint();
        });
    }

    public Command ShootOn() {
        return runOnce(() -> {
            applyShooterSetpoint();
            shooterIsOn = true;
        });
    }

    public Command ShootOff() {
        return runOnce(() -> {
            if (shootermotor != null) shootermotor.setVoltage(0.0);

            shooterIsOn = false;
        });
    }

    /** Spin up wheels, wait for speed, then enable feed. */
    public Command ShooterOn() {
        return IntakeFeedOff()
            .andThen(ShootOn())
            .andThen(new WaitCommand(0.2))
            .andThen(FeedOn());
    }

    /** Cut feed, wait briefly, then spin down wheels. */
    public Command ShooterOff() {
        return FeedOff()
            .andThen(new WaitCommand(0.2))
            .andThen(ShootOff());
    }

    /** Toggle shooter on/off. */
    public Command toggleShooter() {
        return runOnce(() -> {          // fix: was branching outside lambda
            if (shooterIsOn) {
                if (shootermotor != null) shootermotor.setVoltage(0.0);
                moveHood(0).schedule();
                shooterIsOn = false;
            } else {
                applyShooterSetpoint();
                shooterIsOn = true;
            }
        });
    }
/*
     public Command toggleFeed() {
        return runOnce(() -> {          // fix: was branching outside lambda
            if (feedIsOn) {
                FeedOff();
            } else {
                FeedOn();
            }
        });
    }
 */
     public Command toggleFeed() {
        return runOnce(() -> {
            if (feedIsOn) {
                if (feedmotor != null) feedmotor.setVoltage(0.0);
                feedIsOn = false;
            } else {
                if (feedmotor != null) feedmotor.setVoltage(-7);
                    feedIsOn       = true;
                    intakeFeedIsOn = false;
            }
        });
    }

    /** Toggle between intake-feed and off. */
    public Command toggleIntakeFeed() {
        return runOnce(() -> {          // fix: was branching outside lambda
            if (intakeFeedIsOn) {
                if (feedmotor != null) feedmotor.setVoltage(0);
                intakeFeedIsOn = false;
            } else {
                if (feedmotor != null) feedmotor.setVoltage(1.5);
                intakeFeedIsOn = true;
                feedIsOn       = false;
            }
        });
    }

    

    // -------------------------------------------------------------------------
    // Telemetry
    // -------------------------------------------------------------------------

    @Override
    public void periodic() {
        if (hoodEncoder    != null) SmartDashboard.putNumber("Hood",             hoodEncoder.getPosition());
        if (shooterEncoder != null) SmartDashboard.putNumber("Shooter Velocity", shooterEncoder.getVelocity());
        SmartDashboard.putNumber("Shooter Voltage", shootermotor.getAppliedOutput()*shootermotor.getBusVoltage());
        SmartDashboard.putNumber("Shooter1Temp", shootermotor.getMotorTemperature());
        SmartDashboard.putNumber("Shooter2Temp", shootermotor2.getMotorTemperature());
        SmartDashboard.putBoolean("Shooter-FeedIsOn",       feedIsOn);
        SmartDashboard.putBoolean("Shooter-IntakeFeedIsOn", intakeFeedIsOn);
        SmartDashboard.putBoolean("Shooter-ShooterIsOn",    shooterIsOn);
    }
}
