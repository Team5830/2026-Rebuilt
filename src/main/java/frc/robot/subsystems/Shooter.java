package frc.robot.subsystems;
import com.revrobotics.spark.*;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkRelativeEncoderSim;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.function.Supplier;
import swervelib.encoders.SparkMaxAnalogEncoderSwerve;
import swervelib.encoders.SparkMaxEncoderSwerve;
import swervelib.encoders.SwerveAbsoluteEncoder;
import swervelib.parser.PIDFConfig;
import swervelib.telemetry.SwerveDriveTelemetry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;
import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.RobotBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.RelativeEncoder;

public class Shooter extends SubsystemBase {
    SparkMax feedmotor;
    SparkMaxConfig feedConfig;
    SparkMax shootermotor;
    SparkMaxConfig shooterConfig;
    boolean ShooterisOn = false;
    SparkMax hoodmotor;
    SparkMaxConfig hoodConfig;
    SparkClosedLoopController HoodMotorController;
    RelativeEncoder encoder;

    @SuppressWarnings("removal")
    public Shooter(){
        try{
          feedmotor = new SparkMax(Constants.shooter.feedmotor, MotorType.kBrushless);    
          shootermotor = new SparkMax(Constants.shooter.shootermotor, MotorType.kBrushless);
          hoodmotor = new SparkMax(Constants.shooter.hoodmotor, MotorType.kBrushless);
        }
        catch (RuntimeException ex) {
            DriverStation.reportError("Error instantiating Shooter: " + ex.getMessage(), true);
        }
        shooterConfig = new SparkMaxConfig();
        shooterConfig.idleMode(IdleMode.kCoast);
        hoodConfig.idleMode(IdleMode.kBrake);
        // shooterConfig.encoder.countsPerRevolution(2);
        shooterConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(0.003, 00, 0.18);
        var configret  = shootermotor.configure(shooterConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
         if (REVLibError.kOk != configret){
                DriverStation.reportError("Failed to configure left manipulator motor " + configret, true);
            }
        hoodConfig.softLimit
        .forwardSoftLimit(Constants.shooter.ForwardLimit)
        .forwardSoftLimitEnabled(true)
        .reverseSoftLimit(Constants.shooter.ReverseLimit)
        .reverseSoftLimitEnabled(true);
        hoodConfig.closedLoop
          .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
          // Set PID values for position control. We don't need to pass a closed loop
          // slot, as it will default to slot 0.
          .p(Constants.shooter.hoodp)
          .i(Constants.shooter.hoodi)
          .d(Constants.shooter.hoodd)
          .outputRange(-1, 1);
        hoodConfig.smartCurrentLimit(40);
         var configrett = hoodmotor.configure(hoodConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        if (REVLibError.kOk != configrett){
          DriverStation.reportError("Failed to configure elevator motor " + configrett, true);
        }
         HoodMotorController = hoodmotor.getClosedLoopController();
        // Reset the position to 0 to start within the range of the soft limits
        encoder.setPosition(0);
    }

    public Command moveHood(double angle){
       return runOnce(
          ()-> {
          HoodMotorController.setSetpoint(angle, ControlType.kPosition);
          });
    }
    
    public Command FeedOn(){
      return runOnce(
          ()-> {
          feedmotor.setVoltage(0.5);
        });
      }

    public Command FeedOff(){
        return runOnce(
        ()-> {
        feedmotor.setVoltage(0.0);
        });
    }

    public Command ShootOn(){
        return runOnce(
        ()-> {
        shootermotor.getClosedLoopController().setSetpoint( 300, ControlType.kVelocity);
      });
    } 

    public Command ShootOff(){
      return runOnce(
      ()-> {
      shootermotor.setVoltage(0.0);
      });
    }

    public Command ShooterOn(){
      return runOnce(
      ()-> {
      ShootOn().andThen(new WaitCommand(0.2));
      FeedOn();
      });
    }

    public Command ShooterOff(){
      return runOnce(
      ()-> {
      ShootOff().andThen(new WaitCommand(0.2));
      FeedOff();
      });
    }
    public Command toggleShooter(){
      if (ShooterisOn){  
        ShooterisOn = false;
        return runOnce(
          ()-> { ShooterOff(); }
        );
      }
      else {
        ShooterisOn = true;
        return runOnce(
          ()-> { ShooterOn(); }
        );
      }
    }
}