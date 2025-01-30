package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AlgaeHolder;

public class AlgaeHolderSubsystem extends SubsystemBase {
    private final SparkMax pivotMotor;
    private final SparkMax rollingMotor;
    private final DutyCycleEncoder pivotEncoder;
    private final DigitalInput limitSwitch;

    public AlgaeHolderSubsystem() {
        // Initialize Pivot Motor
        pivotMotor = new SparkMax(AlgaeHolder.PIVOT_MOTOR_ID, MotorType.kBrushless);
        SparkMaxConfig pivotConfig = new SparkMaxConfig();
        pivotConfig.encoder.positionConversionFactor(360.0); // Converts rotations to degrees
        pivotConfig.closedLoop.pid(AlgaeHolder.PIVOT_P, AlgaeHolder.PIVOT_I, AlgaeHolder.PIVOT_D);
        pivotMotor.configure(pivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Initialize Rolling Motor
        rollingMotor = new SparkMax(AlgaeHolder.ROLLING_MOTOR_ID, MotorType.kBrushless);
        SparkMaxConfig rollingConfig = new SparkMaxConfig();
        rollingMotor.configure(rollingConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Initialize Pivot Encoder
        pivotEncoder = new DutyCycleEncoder(AlgaeHolder.PIVOT_ENCODER_CHANNEL);

        // Initialize Limit Switch
        limitSwitch = new DigitalInput(AlgaeHolder.LIMIT_SWITCH_CHANNEL);

        // SmartDashboard Setup
        SmartDashboard.putNumber("Pivot P", AlgaeHolder.PIVOT_P);
        SmartDashboard.putNumber("Pivot I", AlgaeHolder.PIVOT_I);
        SmartDashboard.putNumber("Pivot D", AlgaeHolder.PIVOT_D);
        SmartDashboard.putNumber("Rolling Power", AlgaeHolder.ROLLING_POWER);
    }

    public void setPivotPosition(double targetAngle) {
        SmartDashboard.putNumber("Pivot Target Angle", targetAngle);
        pivotMotor.getClosedLoopController().setReference(targetAngle, ControlType.kPosition);
    }

    public void rollMotor(boolean intake) {
        double power = SmartDashboard.getNumber("Rolling Power", AlgaeHolder.ROLLING_POWER);
        rollingMotor.set(intake ? power : -power);
    }

    public void stopRollingMotor() {
        rollingMotor.set(0.0);
    }

    public boolean isLimitSwitchTriggered() {
        return !limitSwitch.get();
    }

    @Override
    public void periodic() {
        if (isLimitSwitchTriggered()) {
            stopRollingMotor();
            SmartDashboard.putBoolean(" algae Limit Switch Triggered", true);
        } else {
            SmartDashboard.putBoolean(" algae Limit Switch Triggered", false);
        }
        SmartDashboard.putNumber("Pivot Angle", pivotEncoder.get());
    }
}
