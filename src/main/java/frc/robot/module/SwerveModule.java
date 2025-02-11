 package frc.robot.module;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;

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

    private final double m_chassisAngularOffset;
    private SwerveModuleState m_desiredState = new SwerveModuleState(0.0, new Rotation2d());

    private long lastUpdateTime = System.currentTimeMillis();

    //  Dynamic PID Variables (Instead of modifying Constants)
    private double driveP = Constants.Drive.KRAKEN_P;
    private double driveI = Constants.Drive.KRAKEN_I;
    private double driveD = Constants.Drive.KRAKEN_D;

    public SwerveModule(int drivingCANId, int turningCANId, double chassisAngularOffset) {
        m_drivingTalonFX = new TalonFX(drivingCANId);
        m_turningSparkMax = new SparkMax(turningCANId, SparkBase.MotorType.kBrushless);
        m_turningEncoder = m_turningSparkMax.getAbsoluteEncoder();
        m_turningPIDController = m_turningSparkMax.getClosedLoopController();
    
        // Use the fixed offset from Constants
        m_chassisAngularOffset = chassisAngularOffset;

        // Display fixed offsets on SmartDashboard for reference
        SmartDashboard.putNumber("Turn Encoder Offset " + m_turningSparkMax.getDeviceId(), m_chassisAngularOffset);
    
        setupSmartDashboard();
        applyConfigs();
    
        System.out.println("Module " + turningCANId + " Encoder Initial Position: " + m_turningEncoder.getPosition());
    }

    private void setupSmartDashboard() {
        SmartDashboard.putNumber("Turn P", Constants.Drive.TURN_P);
        SmartDashboard.putNumber("Turn I", Constants.Drive.TURN_I);
        SmartDashboard.putNumber("Turn D", Constants.Drive.TURN_D);
        SmartDashboard.putNumber("Drive P", driveP);
        SmartDashboard.putNumber("Drive I", driveI);
        SmartDashboard.putNumber("Drive D", driveD);
    }

    private void applyConfigs() {
        TalonFXConfiguration driveConfig = new TalonFXConfiguration();
        driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        driveConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;

        //  Apply initial PID values from variables
        driveConfig.Slot0.kP = driveP;
        driveConfig.Slot0.kI = driveI;
        driveConfig.Slot0.kD = driveD;

        driveConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        m_drivingTalonFX.getConfigurator().apply(driveConfig);

        SparkMaxConfig config = new SparkMaxConfig();
        config
            .inverted(Constants.Drive.TURN_MOTOR_INVERTED)
            .idleMode(SparkMaxConfig.IdleMode.kBrake)
            .smartCurrentLimit(30);

        config.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(Constants.Drive.TURN_P, Constants.Drive.TURN_I, Constants.Drive.TURN_D)
            .positionWrappingEnabled(true)
            .positionWrappingMinInput(0.0)
            .positionWrappingMaxInput(2 * Math.PI);

        config.encoder
            .positionConversionFactor(Constants.Drive.TURN_ENCODER_POSITION_FACTOR)
            .velocityConversionFactor(Constants.Drive.TURN_ENCODER_VELOCITY_FACTOR);

        m_turningSparkMax.configure(config, SparkBase.ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

  
    private void applyDrivePIDConfig() {
        TalonFXConfiguration driveConfig = new TalonFXConfiguration();
        driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        driveConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;

        //  Apply updated PID values from SmartDashboard
        driveConfig.Slot0.kP = driveP;
        driveConfig.Slot0.kI = driveI;
        driveConfig.Slot0.kD = driveD;

        m_drivingTalonFX.getConfigurator().apply(driveConfig);
    }

    public void periodic() {
        if (System.currentTimeMillis() - lastUpdateTime < 100) {
            return;
        }
        lastUpdateTime = System.currentTimeMillis();

        double newDriveP = SmartDashboard.getNumber("Drive P", driveP);
        double newDriveI = SmartDashboard.getNumber("Drive I", driveI);
        double newDriveD = SmartDashboard.getNumber("Drive D", driveD);

        //  If PID values changed, update dynamically
        if (newDriveP != driveP || newDriveI != driveI || newDriveD != driveD) {
            driveP = newDriveP;
            driveI = newDriveI;
            driveD = newDriveD;
            applyDrivePIDConfig(); 
        }

        SmartDashboard.putNumber("Turn Encoder Position " + m_turningSparkMax.getDeviceId(), m_turningEncoder.getPosition());
    }

    public void setDesiredState(SwerveModuleState desiredState) {
        if (desiredState.speedMetersPerSecond == 0) {
            stopMotors();
            return;
        }
    
        // Get the current angle of the module (accounting for inversion)
        double rawAngle = m_turningEncoder.getPosition();
        rawAngle = Constants.Drive.TURN_ENCODER_INVERTED ? -rawAngle : rawAngle;
        
        Rotation2d currentAngle = Rotation2d.fromRadians(rawAngle);
    
        // Optimize the desired state *properly* using the new instance method
        desiredState.optimize(currentAngle);
    
        // Convert speed to TalonFX velocity units and command the drive motor
        m_drivingTalonFX.setControl(
            m_driveVelocityControl.withVelocity(desiredState.speedMetersPerSecond / Constants.Drive.KRAKEN_VELOCITY_CONVERSION)
        );
    
        // Command the turn motor to the optimized angle
        m_turningPIDController.setReference(desiredState.angle.getRadians(), SparkBase.ControlType.kPosition);
    
        // Store the last commanded state
        m_desiredState = desiredState;
    }
    

    private void stopMotors() {
        m_drivingTalonFX.setControl(m_driveVelocityControl.withVelocity(0));
        m_turningPIDController.setReference(m_turningEncoder.getPosition(), SparkBase.ControlType.kPosition);
    }

    public SwerveModulePosition getPosition() {
        double absolutePosition = Constants.Drive.TURN_ENCODER_INVERTED ? -m_turningEncoder.getPosition() : m_turningEncoder.getPosition();
        // Absolute rotation in radians
        double drivePositionMeters = m_drivingTalonFX.getPosition().getValueAsDouble() 
                                    * Constants.Drive.DRIVE_CONVERSION; // Convert to meters
    
        return new SwerveModulePosition(
            drivePositionMeters,
            new Rotation2d(absolutePosition - m_chassisAngularOffset)
        );
    }
    

    public SwerveModuleState getState() {
        return m_desiredState;
    }

    public void resetEncoders() {
        m_drivingTalonFX.setPosition(0);

        // Display the fixed offset on SmartDashboard for reference
        SmartDashboard.putNumber("Turn Encoder Offset " + m_turningSparkMax.getDeviceId(), m_chassisAngularOffset);

        System.out.println("Swerve Module Reset Encoders: Device ID " + m_turningSparkMax.getDeviceId());
    }
}