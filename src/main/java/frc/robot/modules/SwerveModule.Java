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
import frc.robot.Constants;

public class SwerveModule {
    private final String name;
    private final WPI_TalonFX driveMotor;
    private final CANSparkMax turnMotor;
    private final SparkMaxRelativeEncoder turnEncoder;
    private final SparkMaxPIDController turnPIDController;

    private double targetVelocity = 0.0;
    private double targetAngle = 0.0;

    public SwerveModule(String name, int driveID, int turnID, Translation2d location) {
        this.name = name;

        driveMotor = new WPI_TalonFX(driveID);
        driveMotor.configFactoryDefault();
        driveMotor.configSupplyCurrentLimit(
            new SupplyCurrentLimitConfiguration(true, Constants.Drive.DRIVE_CURRENT_LIMIT, 0, 0)
        );
        driveMotor.setSelectedSensorPosition(0);

        turnMotor = new CANSparkMax(turnID, MotorType.kBrushless);
        turnMotor.restoreFactoryDefaults();
        turnMotor.setSmartCurrentLimit(Constants.Drive.TURN_CURRENT_LIMIT);
        turnMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

        turnEncoder = (SparkMaxRelativeEncoder) turnMotor.getEncoder();
        turnEncoder.setPositionConversionFactor(Constants.Drive.TURN_POSITION_CONVERSION);
        turnEncoder.setVelocityConversionFactor(Constants.Drive.TURN_VELOCITY_CONVERSION);
        turnEncoder.setPosition(0);

        turnPIDController = turnMotor.getPIDController();
        turnPIDController.setP(Constants.Drive.TURN_P);
        turnPIDController.setI(Constants.Drive.TURN_I);
        turnPIDController.setD(Constants.Drive.TURN_D);
    }

    public void setTargetState(SwerveModuleState state) {
        SwerveModuleState optimizedState = SwerveModuleState.optimize(
            state, Rotation2d.fromRadians(turnEncoder.getPosition())
        );
        targetVelocity = optimizedState.speedMetersPerSecond;
        targetAngle = optimizedState.angle.getRadians();

        driveMotor.set(TalonFXControlMode.PercentOutput, targetVelocity / Constants.Drive.MAX_SPEED);
        turnPIDController.setReference(targetAngle, ControlType.kPosition);
    }

    public SwerveModuleState getState() {
        double velocity = driveMotor.getSelectedSensorVelocity() * Constants.Drive.DRIVE_VELOCITY_CONVERSION;
        double angle = turnEncoder.getPosition();
        return new SwerveModuleState(velocity, new Rotation2d(angle));
    }

    public SwerveModulePosition getPosition() {
        double position = driveMotor.getSelectedSensorPosition() * Constants.Drive.DRIVE_POSITION_CONVERSION;
        double angle = turnEncoder.getPosition();
        return new SwerveModulePosition(position, new Rotation2d(angle));
    }

    public void resetEncoders() {
        driveMotor.setSelectedSensorPosition(0);
        turnEncoder.setPosition(0);
    }
}
