package frc.robot.subsystems;

import frc.robot.Constants;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.RelativeEncoder;


public class Hopper extends SubsystemBase {
    private SparkMax hoppermotor;
    private SparkMaxConfig hoppermotorConfig;
    public boolean hopperOut = false;
    private RelativeEncoder   encoder;
    public Hopper  () {     
        try{
            hoppermotor = new SparkMax(Constants.hopper.hoppermotor, MotorType.kBrushless);
            hoppermotorConfig = new SparkMaxConfig();
            double ConversionFactor = 1.2;
            encoder = hoppermotor.getEncoder();
            hoppermotorConfig.encoder.positionConversionFactor(ConversionFactor);
            hoppermotorConfig.idleMode(IdleMode.kBrake);
            hoppermotorConfig.softLimit
                .forwardSoftLimit(Constants.hopper.forwardlimit)
                .forwardSoftLimitEnabled(true)
                .reverseSoftLimit(Constants.hopper.reverselimit)
                .reverseSoftLimitEnabled(true);
                hoppermotorConfig.smartCurrentLimit(20);
            var configret = hoppermotor.configure(hoppermotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
            if (REVLibError.kOk != configret){
                DriverStation.reportError("Failed to configure hopper motor " + configret, true);
            }
        } catch (RuntimeException ex) {
            DriverStation.reportError("Error instantiating hopper: " + ex.getMessage(), true);
        }
    }

     public Command toggleHopperCommand(){
        return runOnce(
            ()-> {
            toggleHopper();
        });
    }
    public void toggleHopper() {
        if (hopperOut) {
            retractHopper();
            hopperOut = false;
            System.out.println("Hopper retracted");
        } else{
            extendHopper();
            hopperOut = true;
            System.out.println("Hopper extended");
        }; 
    }

    public void extendHopper() {
        hoppermotor.setVoltage(-6);
    }
    public void retractHopper() {
        hoppermotor.setVoltage(8);
    }

  @Override
  public void periodic()
  {
    SmartDashboard.putNumber("Hopper",encoder.getPosition());
  }
}