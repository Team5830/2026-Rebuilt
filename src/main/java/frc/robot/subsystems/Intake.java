package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

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
    
}
//six 676767676767676767676767676767676767676 seven 676767676767676767676767676767 -Charlotte