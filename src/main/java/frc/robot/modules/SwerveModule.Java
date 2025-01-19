// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.drivetrain;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.Drive;
import frc.robot.Constants.Turn;

public class SwerveModule {
    public final String name;
    private final WPI_TalonFX driveMotor;
    private final CANSparkMax turnMotor;
    private final SparkMaxRelativeEncoder turnEncoder;
    private final SparkMaxPIDController turnPIDController;

    private double targetVelocity = 0.0;
    private double targetAngle = 0.0;

    public SwerveModule(String name, int driveID, int turnID, Translation2d locationOffset) {
        this.name = name;

        // Initialize drive motor (Kraken X60)
        driveMotor = new WPI_TalonFX(driveID);
        driveMotor.configFactoryDefault();
        driveMotor.configSupplyCurrentLimit(
            new SupplyCurrentLimitConfiguration(true, Drive.DRIVE_CURRENT_LIMIT, 0, 0)
        );
        driveMotor.setSelectedSensorPosition(0);

        // Initialize turn motor (NEO with SparkMax)
        turnMotor = new CANSparkMax(turnID, MotorType.kBrushless);
        turnMotor.restoreFactoryDefaults();
        turnMotor.setSmartCurrentLimit(Drive.TURN_CURRENT_LIMIT);
        turnMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

        // Initialize encoder and PID controller
        turnEncoder = (SparkMaxRelativeEncoder) turnMotor.getEncoder();
        turnEncoder.setPositionConversionFactor(Drive.TURN_POSITION_CONVERSION);
        turnEncoder.setVelocityConversionFactor(Drive.TURN_VELOCITY_CONVERSION);
        turnEncoder.setPosition(0);

        turnPIDController = turnMotor.getPIDController();
        turnPIDController.setP(Turn.P);
        turnPIDController.setI(Turn.I);
        turnPIDController.setD(Turn.D);
    }

    public void readHardware() {
        targetVelocity = driveMotor.getSelectedSensorVelocity() * Drive.DRIVE_VELOCITY_CONVERSION;
        targetAngle = turnEncoder.getPosition();
    }

    public void moveTowardsTarget() {
        double currentAngle = turnEncoder.getPosition();
        double optimizedAngle = targetAngle % (2.0 * Math.PI);
        if (optimizedAngle < 0.0) optimizedAngle += 2.0 * Math.PI;

        double newTargetAngle = targetAngle + currentAngle - optimizedAngle;
        if (optimizedAngle - currentAngle > Math.PI) newTargetAngle -= 2.0 * Math.PI;
        else if (optimizedAngle - currentAngle < -Math.PI) newTargetAngle += 2.0 * Math.PI;

        driveMotor.set(TalonFXControlMode.PercentOutput, targetVelocity / Drive.MAX_SPEED);
        turnPIDController.setReference(newTargetAngle, ControlType.kPosition);
    }

    public void setTargetState(SwerveModuleState state) {
        SwerveModuleState optimizedState = SwerveModuleState.optimize(
            state, Rotation2d.fromRadians(turnEncoder.getPosition())
        );
        targetVelocity = optimizedState.speedMetersPerSecond;
        targetAngle = optimizedState.angle.getRadians();
    }

    public SwerveModuleState getState() {
        double velocity = driveMotor.getSelectedSensorVelocity() * Drive.DRIVE_VELOCITY_CONVERSION;
        double angle = turnEncoder.getPosition();
        return new SwerveModuleState(velocity, new Rotation2d(angle));
    }

    public SwerveModulePosition getPosition() {
        double position = driveMotor.getSelectedSensorPosition() * Drive.DRIVE_POSITION_CONVERSION;
        double angle = turnEncoder.getPosition();
        return new SwerveModulePosition(position, new Rotation2d(angle));
    }

    public void resetEncoders() {
        driveMotor.setSelectedSensorPosition(0);
        turnEncoder.setPosition(0);
    }
}
