package frc.robot.module;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class SwerveModule {
    private final TalonFX m_drivingTalonFX;
    private final SparkMax m_turningSparkMax;
    private final AbsoluteEncoder m_turningEncoder;
    private final SparkClosedLoopController m_turningPIDController;
    private final VelocityDutyCycle m_driveVelocityControl = new VelocityDutyCycle(0);
    private double m_chassisAngularOffset;
    private SwerveModuleState m_desiredState;

    public SwerveModule(int drivingCANId, int turningCANId, double chassisAngularOffset) {
        //  Configure Kraken X60 (Drive Motor)
        m_drivingTalonFX = new TalonFX(drivingCANId);

        TalonFXConfiguration driveConfig = new TalonFXConfiguration();
        driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        driveConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        driveConfig.Slot0.kP = Constants.Drive.KRAKEN_P;
        driveConfig.Slot0.kI = Constants.Drive.KRAKEN_I;
        driveConfig.Slot0.kD = Constants.Drive.KRAKEN_D;
        driveConfig.Slot0.kV = Constants.Drive.KRAKEN_KV;
        driveConfig.Slot0.kS = Constants.Drive.KRAKEN_KS;

        //  Apply and persist config
        m_drivingTalonFX.getConfigurator().apply(driveConfig, 50);

        //  Configure SparkMax (Turn Motor) using REVLib 2025
        m_turningSparkMax = new SparkMax(turningCANId, MotorType.kBrushless);
        SparkMaxConfig turnConfig = new SparkMaxConfig();

        turnConfig
            .inverted(Constants.Drive.TURN_MOTOR_INVERTED)
            .idleMode(IdleMode.kBrake);

        //  Setup Absolute Encoder (Turning)
        m_turningEncoder = m_turningSparkMax.getAbsoluteEncoder();
        turnConfig.encoder
            .positionConversionFactor(Constants.Drive.TURN_POSITION_CONVERSION)
            .velocityConversionFactor(Constants.Drive.TURN_VELOCITY_CONVERSION)
            .inverted(Constants.Drive.TURN_ENCODER_INVERTED); // ✅ Set encoder inversion here

        //  Setup PID Controller for Turning
        turnConfig.closedLoop
            .feedbackSensor(ClosedLoopConfig.FeedbackSensor.kPrimaryEncoder)
            .pid(Constants.Turn.P, Constants.Turn.I, Constants.Turn.D)
            .positionWrappingEnabled(true)
            .positionWrappingMinInput(Constants.Drive.TURN_ENCODER_POSITION_PID_MIN_INPUT)
            .positionWrappingMaxInput(Constants.Drive.TURN_ENCODER_POSITION_PID_MAX_INPUT);

        //  Apply config with persistence using REVLib 2025 API
        m_turningSparkMax.configure(turnConfig, SparkBase.ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        m_turningPIDController = m_turningSparkMax.getClosedLoopController();

        System.out.println("Module " + turningCANId + " Encoder Initial Position: " + m_turningEncoder.getPosition());

        // Auto-calibrate the offset on startup
        calibrateModuleOffset(chassisAngularOffset);

        //  Initialize desired state
        m_desiredState = new SwerveModuleState(0.0, new Rotation2d(m_turningEncoder.getPosition()));

        resetEncoders();
    }

    private void calibrateModuleOffset(double storedOffset) {
        double absoluteEncoderPosition = m_turningEncoder.getPosition();
        m_chassisAngularOffset = storedOffset - absoluteEncoderPosition;
        System.out.println("Calibrated Offset for Module " + m_turningSparkMax.getDeviceId() + ": " + m_chassisAngularOffset);
    }

    public void updateDashboard() {
        SmartDashboard.putNumber("Module " + m_turningSparkMax.getDeviceId() + " Encoder", m_turningEncoder.getPosition());
        SmartDashboard.putNumber("Module " + m_turningSparkMax.getDeviceId() + " Offset", m_chassisAngularOffset);
    }

    public SwerveModuleState getState() {
        return m_desiredState;
    }

    public SwerveModulePosition getPosition() {
        double encoderValue = m_turningEncoder.getPosition() - m_chassisAngularOffset;
        SmartDashboard.putNumber("Module " + m_turningSparkMax.getDeviceId() + " Encoder Adjusted", encoderValue);
        
        return new SwerveModulePosition(
            m_drivingTalonFX.getPosition().getValueAsDouble() * Constants.Drive.KRAKEN_POSITION_CONVERSION,
            new Rotation2d(encoderValue)
        );
    }

    public void setDesiredState(SwerveModuleState desiredState) {
        // Create a copy of the desired state to call instance method optimize()
        SwerveModuleState optimizedState = new SwerveModuleState(
            desiredState.speedMetersPerSecond,
            desiredState.angle
        );

        // Apply optimization (new 2025 instance method)
        optimizedState.optimize(new Rotation2d(m_turningEncoder.getPosition()));

        //  Apply optimized state
        m_drivingTalonFX.setControl(m_driveVelocityControl.withVelocity(optimizedState.speedMetersPerSecond));
        m_turningPIDController.setReference(optimizedState.angle.getRadians(), SparkBase.ControlType.kPosition);

        m_desiredState = optimizedState;
    }

    public void resetEncoders() {
        m_drivingTalonFX.setPosition(0); 
    }
}
