package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
    private static final double INTAKE_VOLTAGE = 3.0;
    private static final double FEED_VOLTAGE   = 2.0;

    private final SparkFlex intakeMotor1;
    private final SparkMax  intakeMotor2;
    private boolean intakeIsOn = false;
    private boolean feedIsOn   = false;

    public Intake() {
        SparkFlex m1 = null;
        try {
            m1 = new SparkFlex(Constants.intake.motor1ID, MotorType.kBrushless);
        } catch (RuntimeException ex) {
            DriverStation.reportError("Error instantiating Intake motor 1: " + ex.getMessage(), true);
        }
        SparkMax m2 = null;
        try {
            m2 = new SparkMax(Constants.intake.motor2ID, MotorType.kBrushless);
        } catch (RuntimeException ex) {
            DriverStation.reportError("Error instantiating Intake motor 2: " + ex.getMessage(), true);
        }
        intakeMotor1 = m1;
        intakeMotor2 = m2;
    }

    private void setVoltages(double v1, double v2) {
        if (intakeMotor1 != null) intakeMotor1.setVoltage(v1);
        if (intakeMotor2 != null) intakeMotor2.setVoltage(v2);
    }

    /** Spin intake rollers inward to pick up fuel. Clears feed state. */
    public Command IntakeOn() {
        return runOnce(() -> {
            setVoltages(-INTAKE_VOLTAGE, INTAKE_VOLTAGE);
            intakeIsOn = true;
            feedIsOn   = false;
        });
    }

    public Command IntakeOff() {
        return runOnce(() -> {
            setVoltages(0, 0);
            intakeIsOn = false;
        });
    }

    /** Spin intake rollers to push fuel toward the shooter. Clears intake state. */
    public Command FeedOn() {
        return runOnce(() -> {
            setVoltages(FEED_VOLTAGE, FEED_VOLTAGE);
            feedIsOn   = true;
            intakeIsOn = false;
        });
    }

    public Command FeedOff() {
        return runOnce(() -> {
            setVoltages(0, 0);
            feedIsOn = false;
        });
    }

    public Command toggleIntake() {
        return runOnce(() -> {
            if (intakeIsOn) {
                setVoltages(0, 0);
                intakeIsOn = false;
            } else {
                setVoltages(-INTAKE_VOLTAGE, INTAKE_VOLTAGE);
                intakeIsOn = true;
                feedIsOn   = false;
            }
        });
    }

    public Command toggleFeedMode() {
        return runOnce(() -> {
            if (feedIsOn) {
                setVoltages(0, 0);
                feedIsOn = false;
            } else {
                setVoltages(FEED_VOLTAGE, FEED_VOLTAGE);
                feedIsOn   = true;
                intakeIsOn = false;
            }
        });
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Intake-feedIsOn",   feedIsOn);
        SmartDashboard.putBoolean("Intake-intakeIsOn", intakeIsOn);
    }
}
