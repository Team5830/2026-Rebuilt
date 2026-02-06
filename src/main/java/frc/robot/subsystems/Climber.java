package frc.robot.subsystems;
import frc.robot.Constants.climber;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.LimitSwitchConfig.Type;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.EncoderConfig;


public class Climber extends SubsystemBase {
     private SparkMax climbermotor;
    private SparkMaxConfig climbermotorConfig;
    private SparkLimitSwitch forwardLimitSwitch;
    private SparkLimitSwitch reverseLimitSwitch;
    private RelativeEncoder encoder;
    private SparkClosedLoopController climbermotorController;
    private EncoderConfig encoderconfig; 
    public Climber  () {
      try{
        climbermotor = new SparkMax(Constants.climber.climbermotor, MotorType.kBrushless);
        climbermotorConfig = new SparkMaxConfig();
        forwardLimitSwitch = climbermotor.getForwardLimitSwitch();
        reverseLimitSwitch = climbermotor.getReverseLimitSwitch();
        encoder = climbermotor.getEncoder();
        double ConversionFactor = 1.2;
        climbermotorConfig.encoder.positionConversionFactor(ConversionFactor);
         climbermotorConfig.softLimit
            .forwardSoftLimit(Constants.climber.forwardlimit)
            .forwardSoftLimitEnabled(true)
            .reverseSoftLimit(Constants.climber.reverselimit)
            .reverseSoftLimitEnabled(true);
              climbermotorConfig.smartCurrentLimit(40);
         var configret = climbermotor.configure(climbermotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        if (REVLibError.kOk != configret){
          DriverStation.reportError("Failed to configure climber motor " + configret, true);}
      } catch (RuntimeException ex) {DriverStation.reportError("Error instantiating climber: " + ex.getMessage(), true);}
    }
     public Command Up(){
    return runOnce(
        ()-> {
          climbermotor.setVoltage(0.3);
      });
    }
    public Command Down(){
    return runOnce(
        ()-> {
          climbermotor.setVoltage(-0.3);
      });
    }



        

    



}
