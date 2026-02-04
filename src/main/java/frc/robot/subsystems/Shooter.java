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

public class Shooter extends SubsystemBase {
    SparkFlex feedmotor;
    SparkMaxConfig feedConfig;
    SparkFlex shootermotor;
    SparkMaxConfig shooterConfig;

    @SuppressWarnings("removal")
    public Shooter(){
        try{
          feedmotor = new SparkFlex(Constants.shooter.feedmotor, MotorType.kBrushless);    
          shootermotor = new SparkFlex(Constants.shooter.shootermotor, MotorType.kBrushless);
        }
        catch (RuntimeException ex) {
            DriverStation.reportError("Error instantiating Shooter: " + ex.getMessage(), true);
        }
        shooterConfig = new SparkMaxConfig();
        shooterConfig.idleMode(IdleMode.kCoast);
        shooterConfig.encoder.countsPerRevolution(2);
        shooterConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(0.1, 00, 00);
        var configret  = shootermotor.configure(shooterConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
         if (REVLibError.kOk != configret){
                DriverStation.reportError("Failed to configure left manipulator motor " + configret, true);
            }
            
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

}