package frc.robot.module;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig; 

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
    private final double m_chassisAngularOffset;
    private SwerveModuleState m_desiredState = new SwerveModuleState(0.0, new Rotation2d());

    public SwerveModule(int drivingCANId, int turningCANId, double chassisAngularOffset) {
      
        m_drivingTalonFX = new TalonFX(drivingCANId);
        TalonFXConfiguration driveConfig = new TalonFXConfiguration();
        driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        driveConfig.MotorOutput.Inverted = Constants.Drive.KRAKEN_INVERTED;

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


        m_turningEncoder = m_turningSparkMax.getAbsoluteEncoder();
        m_turningPIDController = m_turningSparkMax.getClosedLoopController();

        m_chassisAngularOffset = chassisAngularOffset;
        m_desiredState.angle = new Rotation2d(m_turningEncoder.getPosition());
    }

    public SwerveModuleState getState() {
        return m_desiredState;
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            m_drivingTalonFX.getPosition().getValueAsDouble() * Constants.Drive.KRAKEN_POSITION_CONVERSION,
            new Rotation2d((m_turningEncoder.getPosition() * Constants.Drive.TURN_POSITION_CONVERSION) - m_chassisAngularOffset)
        );
    }

    public void setDesiredState(SwerveModuleState desiredState) {
       
        SwerveModuleState correctedDesiredState = new SwerveModuleState(
            desiredState.speedMetersPerSecond,
            desiredState.angle.plus(Rotation2d.fromRadians(m_chassisAngularOffset))
        );

        Rotation2d currentAngle = new Rotation2d(m_turningEncoder.getPosition() * Constants.Drive.TURN_POSITION_CONVERSION);
        Rotation2d delta = correctedDesiredState.angle.minus(currentAngle);

        if (Math.abs(delta.getRadians()) > Math.PI / 2) {
            correctedDesiredState = new SwerveModuleState(
                -correctedDesiredState.speedMetersPerSecond,
                correctedDesiredState.angle.plus(Rotation2d.fromRadians(Math.PI))
            );
        }

        m_drivingTalonFX.setControl(m_driveVelocityControl.withVelocity(correctedDesiredState.speedMetersPerSecond));

        m_turningPIDController.setReference(
            correctedDesiredState.angle.getRadians(),
            SparkBase.ControlType.kPosition
        );

        m_desiredState = correctedDesiredState;
    }

    public void resetEncoders() {
        m_drivingTalonFX.setPosition(0);
        m_turningSparkMax.getEncoder().setPosition(0);
    }
}
