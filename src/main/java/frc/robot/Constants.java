// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;

public final class Constants {
    public static class Drive {
        // Physical constants
        public static final double WHEEL_DIAMETER = 0.1016; // 4 inches in meters (VERIFY for Kraken wheels)
        public static final double DRIVE_REDUCTION = 8.33; // Gear reduction for drive motor (VERIFY for Kraken)
        public static final double DRIVE_CONVERSION = WHEEL_DIAMETER * Math.PI / DRIVE_REDUCTION; // Meters per rotation
        public static final double MAX_SPEED = 3.0; // Maximum linear speed (m/s)
        public static final double MAX_ANGULAR_SPEED = Math.PI; // Maximum angular speed (rad/s)

        // Kraken X60 TalonFX Encoder (2048 CPR)
        public static final double DRIVE_TICKS_PER_ROTATION = 2048.0; // Native units for Kraken X60
        public static final double DRIVE_POSITION_CONVERSION = DRIVE_CONVERSION / DRIVE_TICKS_PER_ROTATION; // Ticks to meters
        public static final double DRIVE_VELOCITY_CONVERSION = DRIVE_POSITION_CONVERSION * 10.0; // Ticks per 100ms to m/s

        // SparkMax NEO Encoder for turning
        public static final double TURN_GEAR_RATIO = 12.0; // Gear reduction for turn motor (VERIFY for your modules)
        public static final double TURN_POSITION_CONVERSION = 2 * Math.PI / TURN_GEAR_RATIO; // Radians per rotation
        public static final double TURN_VELOCITY_CONVERSION = TURN_POSITION_CONVERSION / 60.0; // RPM to rad/s

        // Motor current limits
        public static final int DRIVE_CURRENT_LIMIT = 35; // Current limit for Kraken X60
        public static final int TURN_CURRENT_LIMIT = 20; // Current limit for SparkMax NEO

        // PID coefficients for driving motor
        public static final double DRIVE_P = 0.05; // Proportional gain for driving
        public static final double DRIVE_I = 0.0;  // Integral gain for driving
        public static final double DRIVE_D = 0.0;  // Derivative gain for driving

        // PID coefficients for turning motor
        public static final double TURN_P = 1.5; // Proportional gain for turning
        public static final double TURN_I = 0.0; // Integral gain for turning
        public static final double TURN_D = 0.5; // Derivative gain for turning
    }

    public static class Turn {
        public static final double P = Drive.TURN_P;
        public static final double I = Drive.TURN_I;
        public static final double D = Drive.TURN_D;
    }

    public static class Intake {
        public static final int HORIZONTAL_ROLLER_MOTOR_ID = 13;
        public static final int LEFT_VERTICAL_ROLLER_MOTOR_ID = 14;
        public static final int RIGHT_VERTICAL_ROLLER_MOTOR_ID = 15;
        public static final int ROTATION_MOTOR_ID = 12;

        public static final int BORE_ENCODER_CHANNEL = 1;

        public static final double DEFAULT_ROLLER_POWER = 0.8;

        public static final double ROTATION_P = 0.1;
        public static final double ROTATION_I = 0.0;
        public static final double ROTATION_D = 0.0;

        public static final double ROTATION_MAX_ANGLE = 160.0;
        public static final double ROTATION_TOLERANCE = 5.0;

        public static final int MOTOR_CURRENT_LIMIT = 40;
    }

    public static class Elevator {
        public static final int LEFT_MOTOR_ID = 9;
        public static final int RIGHT_MOTOR_ID = 10;

        public static final double ENCODER_CONVERSION_FACTOR = 0.01;

        public static final double TOLERANCE = 0.05;

        public static final double INTAKE_POSITION = 0.0;
        public static final double REEF_LEVEL_1 = 1.0;
        public static final double REEF_LEVEL_2 = 1.5;
        public static final double REEF_LEVEL_3 = 2.0;
        public static final double REEF_LEVEL_4 = 2.5;

        public static final double kP = 0.1;
        public static final double kI = 0.0;
        public static final double kD = 0.0;

        public static final int LIMIT_SWITCH_CHANNEL = 0;
    }

    public static class Shooter {
        public static final int MOTOR_ID = 16;
        public static final int LIMIT_SWITCH_CHANNEL = 2;

        public static final double FORWARD_POWER = 1.0;
        public static final double REVERSE_POWER = -1.0;
        public static final double STOP_POWER = 0.0;
    }

    public static class AlgaeHolder {
        // Motor CAN IDs
        public static final int PIVOT_MOTOR_ID = 17;
        public static final int TOP_ROLLER_MOTOR_ID = 18;

        // Encoder DIO Channel
        public static final int PIVOT_ENCODER_CHANNEL = 4;

        // Limit Switch
        public static final int LIMIT_SWITCH_CHANNEL = 5;

        // Positions (degrees)
        public static final double FULLY_FOLDED_POSITION = 0.0;
        public static final double ALGAE_INTAKE_POSITION = 90.0;
        public static final double HORIZONTAL_POSITION = 180.0;

        // Pivot PID Coefficients
        public static final double PIVOT_P = 0.1;
        public static final double PIVOT_I = 0.0;
        public static final double PIVOT_D = 0.0;

        // Rolling Motor Power
        public static final double TOP_ROLLER_POWER = 0.5;
    }

    public enum SwerveModules {
        FRONTLEFT(2, 1, 3, new Translation2d(0.381, 0.381), -(0.0431 + 0.39216 + Math.PI - (2 * Math.PI))),
        FRONTRIGHT(3, 2, 4, new Translation2d(0.381, -0.381), -(1.57568)),
        BACKLEFT(0, 5, 6, new Translation2d(-0.381, 0.381), -(-0.97338 + 6.2831)),
        BACKRIGHT(1, 7, 8, new Translation2d(-0.381, -0.381), -(0.278867));

        private final int encoderPort;
        private final int driveMotorID;
        private final int turnMotorID;
        private final Translation2d moduleLocation;
        private final double encoderOffset;

        SwerveModules(int encoderPort, int driveMotorID, int turnMotorID, Translation2d moduleLocation, double encoderOffset) {
            this.encoderPort = encoderPort;
            this.driveMotorID = driveMotorID;
            this.turnMotorID = turnMotorID;
            this.moduleLocation = moduleLocation;
            this.encoderOffset = encoderOffset;
        }

        public int getEncoderPort() {
            return encoderPort;
        }

        public int getDriveMotorID() {
            return driveMotorID;
        }

        public int getTurnMotorID() {
            return turnMotorID;
        }

        public Translation2d getModuleLocation() {
            return moduleLocation;
        }

        public double getEncoderOffset() {
            return encoderOffset;
        }
    }
}
