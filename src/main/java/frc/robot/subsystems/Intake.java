package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.Lights;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

public class Intake extends SubsystemBase {
    SparkFlex intakeMotor1, intakeMotor2;
    SparkMaxConfig config1, config2;
    boolean IntakeisOn = false;
    Lights m_Lights;
    boolean FeedisOn=false;
    public Intake(){
        try{
            intakeMotor1 = new SparkFlex(Constants.intake.motor1ID,MotorType.kBrushless);
            intakeMotor2 = new SparkFlex(Constants.intake.motor2ID,MotorType.kBrushless);
        } catch (RuntimeException ex) {
            DriverStation.reportError("Error instantiating Intake Motors: " + ex.getMessage(), true);
        }
    }

     public Command IntakeOn(){
        return runOnce(
            ()-> {
            intakeMotor1.setVoltage(0.3);
            intakeMotor2.setVoltage(-0.3);
          });
      }
    public Command IntakeOff(){
        return runOnce(
            ()-> {
            intakeMotor1.setVoltage(0.0);
            intakeMotor2.setVoltage(0.0);
          });
        }
    public Command FeedOn(){
        return runOnce(
            ()-> {
            intakeMotor1.setVoltage(0.3);
            intakeMotor2.setVoltage(0.3);
          });
      }
    public Command FeedOff(){
        return runOnce(
            ()-> {
            intakeMotor1.setVoltage(0.0);
            intakeMotor2.setVoltage(0.0);
          });
      }
    
    public Command toggleIntake(){
      if (IntakeisOn){  
        IntakeisOn = false;
        return runOnce(
          ()-> { IntakeOff();}
        );
      }
      else {
        IntakeisOn = true;
        return runOnce(
          ()-> { IntakeOn(); }
        );
      }
    }
     public Command toggleFeed(){
      if (FeedisOn){  
        FeedisOn = false;
        return runOnce(
          ()-> { FeedOff(); }
        );
      }
      else {
        FeedisOn = true;
        return runOnce(
          ()-> { FeedOn(); }
        );
      }
    }
}
