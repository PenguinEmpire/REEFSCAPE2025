package frc.robot.module;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;

public class SwerveModule {
    private final TalonFX m_drivingTalonFX;
    private final SparkMax m_turningSparkMax;
    private final RelativeEncoder m_turningRelativeEncoder;
    private final SparkClosedLoopController m_turningPIDController;
    private final VelocityDutyCycle m_driveVelocityControl = new VelocityDutyCycle(0);
    private final double m_chassisAngularOffset;
    private SwerveModuleState m_desiredState = new SwerveModuleState(0.0, new Rotation2d());

    public SwerveModule(int drivingCANId, int turningCANId, double chassisAngularOffset) {

        m_drivingTalonFX = new TalonFX(drivingCANId);
        TalonFXConfiguration driveConfig = new TalonFXConfiguration();

    
        driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        driveConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;

 
        var slot0Configs = driveConfig.Slot0;
        slot0Configs.kP = Constants.Drive.KRAKEN_P;
        slot0Configs.kI = Constants.Drive.KRAKEN_I;
        slot0Configs.kD = Constants.Drive.KRAKEN_D;
        slot0Configs.kV = Constants.Drive.KRAKEN_KV;
        slot0Configs.kS = Constants.Drive.KRAKEN_KS;

        
        m_drivingTalonFX.getConfigurator().apply(driveConfig, 50);

    
        m_turningSparkMax = new SparkMax(turningCANId, MotorType.kBrushless);
        SparkMaxConfig turnConfig = new SparkMaxConfig();
        turnConfig.inverted(Constants.Drive.TURN_MOTOR_INVERTED);
        turnConfig.idleMode(SparkBaseConfig.IdleMode.kBrake);


        m_turningRelativeEncoder = m_turningSparkMax.getEncoder();
        turnConfig.encoder
            .positionConversionFactor(Constants.Drive.TURN_POSITION_CONVERSION)
            .velocityConversionFactor(Constants.Drive.TURN_VELOCITY_CONVERSION);

     
        turnConfig.closedLoop
            .feedbackSensor(ClosedLoopConfig.FeedbackSensor.kPrimaryEncoder)
            .pid(Constants.Turn.P, Constants.Turn.I, Constants.Turn.D)
            .positionWrappingEnabled(true)
            .positionWrappingMinInput(Constants.Drive.TURN_ENCODER_POSITION_PID_MIN_INPUT)
            .positionWrappingMaxInput(Constants.Drive.TURN_ENCODER_POSITION_PID_MAX_INPUT);

        m_turningSparkMax.configure(turnConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
        m_turningPIDController = m_turningSparkMax.getClosedLoopController();

 
        m_chassisAngularOffset = chassisAngularOffset;
        m_desiredState.angle = new Rotation2d(m_turningRelativeEncoder.getPosition());
    }

    /**
     * Get the current state of the swerve module.
     */
    public SwerveModuleState getState() {
        return m_desiredState;
    }

    /**
     * Get the current position of the swerve module.
     * - Uses the **Kraken X60 absolute encoder** for the drive motor.
     * - Uses the **SparkMax relative encoder** for the turning motor.
     */
    public SwerveModulePosition getPosition() {
        // Read from the **Kraken X60's absolute encoder**
        double drivePosition = m_drivingTalonFX.getPosition().getValueAsDouble() * Constants.Drive.KRAKEN_POSITION_CONVERSION;
        
        // Read from the **SparkMax's relative encoder**
        double turnPosition = m_turningRelativeEncoder.getPosition() * Constants.Drive.TURN_POSITION_CONVERSION;

        return new SwerveModulePosition(
            drivePosition,
            new Rotation2d(turnPosition - m_chassisAngularOffset)
        );
    }

    /**
     * Set the desired state for the swerve module.
     */
    public void setDesiredState(SwerveModuleState desiredState) {
        // Apply the chassis angular offset correction
        SwerveModuleState correctedDesiredState = new SwerveModuleState(
            desiredState.speedMetersPerSecond,
            desiredState.angle.plus(Rotation2d.fromRadians(m_chassisAngularOffset))
        );

        // Set Drive Motor Speed using Velocity Control
        m_drivingTalonFX.setControl(m_driveVelocityControl.withVelocity(correctedDesiredState.speedMetersPerSecond));

        // Set Turning Motor Angle using PID (Relative Encoder)
        m_turningPIDController.setReference(
            correctedDesiredState.angle.getRadians(),
            SparkBase.ControlType.kPosition
        );

        // Store the desired state
        m_desiredState = correctedDesiredState;
    }

    /**
     * Reset the encoders for both drive and turning motors.
     */
    public void resetEncoders() {
        // The Kraken X60 (TalonFX) has an **absolute encoder**, so no need to reset it.
        // Reset only the relative encoder of the turning motor.
        m_turningSparkMax.getEncoder().setPosition(0);
    }
}
