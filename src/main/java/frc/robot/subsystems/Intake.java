package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class Intake extends SubsystemBase {
    private static final double INTAKE_VOLTAGE = 2.0;
    private static final double FEED_VOLTAGE   = 6;

    private final SparkFlex intakeMotor1;
    private final SparkMax intakeMotor2;
    private boolean intakeIsOn = false;
    private boolean feedIsOn   = false; 

    public Intake() {
        SparkFlex m1 = null;
        try {
            m1 = new SparkFlex(Constants.intake.motor1ID, MotorType.kBrushless);
        } catch (RuntimeException ex) {
            DriverStation.reportError("Error instantiating Intake Motors: " + ex.getMessage(), true);
        }
        SparkMax m2 = null;
        try {
            m2 = new SparkMax(Constants.intake.motor2ID, MotorType.kBrushless);
        } catch (RuntimeException ex) {
            DriverStation.reportError("Error instantiating Intake Motors: " + ex.getMessage(), true);
        }
        intakeMotor1 = m1;
        intakeMotor2 = m2;
    }

    private void setVoltages(double v1, double v2) {
        if (intakeMotor1 != null) intakeMotor1.setVoltage(v1);
        if (intakeMotor2 != null) intakeMotor2.setVoltage(v2);
    }

    public Command IntakeOn()  { return runOnce(() -> setVoltages( -INTAKE_VOLTAGE, INTAKE_VOLTAGE)); }
    public Command IntakeOff() { return runOnce(() -> setVoltages(0, 0)); }
    public Command FeedOn()    { return runOnce(() -> setVoltages( FEED_VOLTAGE,   FEED_VOLTAGE)); }
    public Command IntakeFeed()    { return runOnce(() -> setVoltages( -FEED_VOLTAGE,   -FEED_VOLTAGE)); }
    public Command FeedOff()   { return runOnce(() -> setVoltages(0, 0)); }

    public Command toggleIntake() {
        Command returncmd;
        if (intakeIsOn){
            returncmd = new SequentialCommandGroup(IntakeOff(), FeedOff());
        }
        else{
            returncmd = new SequentialCommandGroup(IntakeOn(), IntakeFeed());
        }
        intakeIsOn = !intakeIsOn;
        return returncmd;
    }

    public Command toggleFeed() {
        feedIsOn = !feedIsOn;
        return feedIsOn ? FeedOn() : FeedOff();
    }
}
