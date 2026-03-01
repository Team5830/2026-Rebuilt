package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {

    private final SparkFlex intakeMotor1;
    private final SparkFlex intakeMotor2;

    private boolean intakeIsOn = false;
    private boolean feedIsOn   = false;

    public Intake() {
        SparkFlex m1 = null, m2 = null;
        try {
            m1 = new SparkFlex(Constants.intake.motor1ID, MotorType.kBrushless);
            m2 = new SparkFlex(Constants.intake.motor2ID, MotorType.kBrushless);
        } catch (RuntimeException ex) {
            DriverStation.reportError("Error instantiating Intake Motors: " + ex.getMessage(), true);
        }
        intakeMotor1 = m1;
        intakeMotor2 = m2;
    }

    public Command IntakeOn() {
        return runOnce(() -> {
            intakeMotor1.setVoltage(0.3);
            intakeMotor2.setVoltage(-0.3);
        });
    }

    public Command IntakeOff() {
        return runOnce(() -> {
            intakeMotor1.setVoltage(0.0);
            intakeMotor2.setVoltage(0.0);
        });
    }

    public Command FeedOn() {
        return runOnce(() -> {
            intakeMotor1.setVoltage(0.3);
            intakeMotor2.setVoltage(0.3);
        });
    }

    public Command FeedOff() {
        return runOnce(() -> {
            intakeMotor1.setVoltage(0.0);
            intakeMotor2.setVoltage(0.0);
        });
    }

    public Command toggleIntake() {
        intakeIsOn = !intakeIsOn;
        return intakeIsOn ? IntakeOn() : IntakeOff();
    }

    public Command toggleFeed() {
        feedIsOn = !feedIsOn;
        return feedIsOn ? FeedOn() : FeedOff();
    }
}
