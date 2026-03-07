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
import com.revrobotics.spark.SparkBase;

public class Shooter extends SubsystemBase {

    private final SparkMax     feedmotor;
    private final SparkFlex     shootermotor;
    private final SparkFlex     shootermotor2;
    private final SparkMax     hoodmotor;
    private final SparkClosedLoopController hoodController;
    private final RelativeEncoder           hoodEncoder, shooterEncoder;

    private double  shootspeed  = 4200;
    private boolean shooterIsOn = false;
    private boolean feedIsOn = false;

    @SuppressWarnings("removal")
    public Shooter() {
        SparkMax feed = null, hood = null;
        SparkFlex shoot1 = null, shoot2 = null;
        try {
            feed   = new SparkMax(Constants.shooter.feedmotor,    MotorType.kBrushless);
            shoot1 = new SparkFlex(Constants.shooter.shootermotor, MotorType.kBrushless);
            shoot2 = new SparkFlex(Constants.shooter.shootermotor2, MotorType.kBrushless);
            hood   = new SparkMax(Constants.shooter.hoodmotor,    MotorType.kBrushless);
        } catch (RuntimeException ex) {
            DriverStation.reportError("Error instantiating Shooter: " + ex.getMessage(), true);
        }
        feedmotor     = feed;
        shootermotor  = shoot1;
        shootermotor2 = shoot2;
        hoodmotor     = hood;

        // Shooter motor config
        SparkFlexConfig shooterConfig = new SparkFlexConfig();
        shooterConfig.idleMode(IdleMode.kCoast);
        shooterConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(0.003, 0.0, 0.18);
        configureMotorFlex(shootermotor, shooterConfig, "shooter");
        shooterEncoder = shootermotor.getEncoder();

        SparkMaxConfig feedConfig = new SparkMaxConfig();
        feedConfig.idleMode(IdleMode.kCoast);
        configureMotor(feed, feedConfig, "feed");
        // Follower motor config
        SparkFlexConfig shooter2Config = new SparkFlexConfig();
        shooter2Config.idleMode(IdleMode.kCoast);
        shooter2Config.follow(shootermotor, true);
        configureMotorFlex(shootermotor2, shooter2Config, "shooter2");

        // Hood motor config
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
        hoodConfig.smartCurrentLimit(40);
        configureMotor(hoodmotor, hoodConfig, "hood");

        hoodController = hoodmotor.getClosedLoopController();
        hoodEncoder    = hoodmotor.getEncoder();
        hoodEncoder.setPosition(0);
    }

    /** Configure a motor and report any error. */
    private void configureMotor(SparkMax motor, SparkMaxConfig config, String name) {
        if (motor == null) return;
        REVLibError err = motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        if (err != REVLibError.kOk) {
            DriverStation.reportError("Failed to configure " + name + " motor: " + err, true);
        }
    }
    private void configureMotorFlex(SparkFlex motor, SparkFlexConfig config, String name) {
        if (motor == null) return;
        REVLibError err = motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        if (err != REVLibError.kOk) {
            DriverStation.reportError("Failed to configure " + name + " motor: " + err, true);
        }     
    }

    public Command moveHood(double position) {
        return runOnce(() -> hoodController.setSetpoint(position, ControlType.kPosition));
    }

    public Command FeedOn() {
        return runOnce(() -> feedmotor.setVoltage(-6));
    }

    public Command IntakeFeed() {
        return runOnce(() -> feedmotor.setVoltage(3));
    }

    public Command FeedOff() {
        return runOnce(() -> feedmotor.setVoltage(0.0));
    }

    public Command setShootSpeed(double setpoint) {
        
        if (shooterIsOn) {
            return runOnce(() -> {shootspeed = setpoint; ShootOn();});    
        }
        return runOnce(() -> shootspeed = setpoint);
    }

    public Command ShootOn() {
        return runOnce(() ->
            shootermotor.getClosedLoopController().setSetpoint(-shootspeed, ControlType.kVelocity));
    }

    public Command ShootOff() {
        return runOnce(() -> shootermotor.setVoltage(0.0));
    }

    /**
     * Start the shooter: spin up wheels, wait briefly, then enable the feed roller.
     * NOTE: Fixed broken chaining — commands inside runOnce() were being built but never scheduled.
     */
    public Command ShooterOn() {
        return ShootOn().andThen(new WaitCommand(0.2)).andThen(FeedOn());
    }

    /**
     * Stop the shooter: cut the feed, wait briefly, then spin down wheels.
     */
    public Command ShooterOff() {
        return FeedOff().andThen(new WaitCommand(0.2)).andThen(ShootOff());
    }

    /** Toggle shooter on/off. */
    public Command toggleShooter() {
        shooterIsOn = !shooterIsOn;
        return shooterIsOn ? ShooterOn() : ShooterOff();
    }

     public Command toggleIntakeFeed() {
        feedIsOn = !feedIsOn;
        return feedIsOn ? IntakeFeed() : FeedOff();
    }

    @Override
  public void periodic()
  {
    SmartDashboard.putNumber("Hood",hoodEncoder.getPosition());
    SmartDashboard.putNumber("Shooter Velocity",shooterEncoder.getVelocity());
  }
}
