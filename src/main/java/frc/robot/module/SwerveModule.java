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
        m_drivingTalonFX.getConfigurator().apply(new TalonFXConfiguration(), 50);

        TalonFXConfiguration driveConfig = new TalonFXConfiguration();
        driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        driveConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        driveConfig.Slot0.kP = Constants.Drive.KRAKEN_P;
        driveConfig.Slot0.kI = Constants.Drive.KRAKEN_I;
        driveConfig.Slot0.kD = Constants.Drive.KRAKEN_D;
        driveConfig.Slot0.kV = Constants.Drive.KRAKEN_KV;
        driveConfig.Slot0.kS = Constants.Drive.KRAKEN_KS;
        m_drivingTalonFX.getConfigurator().apply(driveConfig, 50);

        //  Configure SparkMax (Turn Motor)
        m_turningSparkMax = new SparkMax(turningCANId, MotorType.kBrushless);
        SparkMaxConfig turnConfig = new SparkMaxConfig(); 

        turnConfig.inverted(Constants.Drive.TURN_MOTOR_INVERTED);
        turnConfig.idleMode(IdleMode.kBrake);

        //  Setup Absolute Encoder (Turning)
        m_turningEncoder = m_turningSparkMax.getAbsoluteEncoder();
        turnConfig.encoder
            .positionConversionFactor(Constants.Drive.TURN_POSITION_CONVERSION)
            .velocityConversionFactor(Constants.Drive.TURN_VELOCITY_CONVERSION);

        //  Setup PID Controller for Turning
        turnConfig.closedLoop
            .feedbackSensor(ClosedLoopConfig.FeedbackSensor.kPrimaryEncoder)
            .pid(Constants.Turn.P, Constants.Turn.I, Constants.Turn.D)
            .positionWrappingEnabled(true)
            .positionWrappingMinInput(Constants.Drive.TURN_ENCODER_POSITION_PID_MIN_INPUT)
            .positionWrappingMaxInput(Constants.Drive.TURN_ENCODER_POSITION_PID_MAX_INPUT);

        //  Apply Configurations
        m_turningSparkMax.configure(turnConfig, SparkBase.ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_turningPIDController = m_turningSparkMax.getClosedLoopController();

        // 🔹 **New Feature: Automatic Calibration on Boot**
        calibrateModuleOffset(chassisAngularOffset);

        // Initialize desired state
        m_desiredState = new SwerveModuleState(0.0, new Rotation2d(m_turningEncoder.getPosition()));

        //  Reset ONLY the drive encoder
        resetEncoders();
    }

    /** 🔹 **Calibrates the encoder offset automatically on boot** */
    private void calibrateModuleOffset(double storedOffset) {
        double absoluteEncoderPosition = m_turningEncoder.getPosition();
        m_chassisAngularOffset = storedOffset - absoluteEncoderPosition;

        // 🟢 **Log the offset for debugging**
        System.out.println("Calibrated Offset for Module: " + m_chassisAngularOffset);
    }

    // 🔹 **Fix: Use m_desiredState in getState()**
    public SwerveModuleState getState() {
        return m_desiredState;
    }

    // Get the current position of the swerve module. 
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            m_drivingTalonFX.getPosition().getValueAsDouble() * Constants.Drive.KRAKEN_POSITION_CONVERSION,
            new Rotation2d(m_turningEncoder.getPosition() - m_chassisAngularOffset)
        );
    }

    // Set desired state of the swerve module.
    public void setDesiredState(SwerveModuleState desiredState) {
        SwerveModuleState correctedState = new SwerveModuleState(
            desiredState.speedMetersPerSecond,
            desiredState.angle.plus(Rotation2d.fromRadians(m_chassisAngularOffset))
        );

        //  Optimize to minimize unnecessary turns
        correctedState.optimize(new Rotation2d(m_turningEncoder.getPosition()));

        //  Drive Motor Control
        m_drivingTalonFX.setControl(m_driveVelocityControl.withVelocity(correctedState.speedMetersPerSecond));

        //  Turning Motor Position Control
        m_turningPIDController.setReference(correctedState.angle.getRadians(), SparkBase.ControlType.kPosition);

        // ✅ Fix: Store the desired state so it is used in `getState()`
        m_desiredState = correctedState;
    }

    /** Reset the drive encoder */
    public void resetEncoders() {
        m_drivingTalonFX.setPosition(0);
    }
}
