// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;

public final class Constants {
    public static class Drive {
        // Physical constants
        public static final double WHEEL_DIAMETER = 0.1016; // 4 inches in meters
        public static final double DRIVE_REDUCTION = 8.33; // Gear reduction for drive motor
        public static final double DRIVE_CONVERSION = WHEEL_DIAMETER * Math.PI / DRIVE_REDUCTION; // Rotations to meters
        public static final double MAX_SPEED = 3.0; // Maximum linear speed (m/s)
        public static final double MAX_ANGULAR_SPEED = Math.PI; // Maximum angular speed (rad/s)

        // TalonFX Integrated Encoder (2048 CPR)
        public static final double DRIVE_TICKS_PER_ROTATION = 2048.0;

        // Conversion for TalonFX encoder
        public static final double DRIVE_POSITION_CONVERSION = DRIVE_CONVERSION / DRIVE_TICKS_PER_ROTATION; // Ticks to meters
        public static final double DRIVE_VELOCITY_CONVERSION = DRIVE_POSITION_CONVERSION * 10.0; // Ticks per 100ms to m/s

        // NEO Integrated Encoder (SparkMax)
        public static final double TURN_GEAR_RATIO = 12.0; // Example gear reduction for turn motor
        public static final double TURN_POSITION_CONVERSION = 2 * Math.PI / TURN_GEAR_RATIO; // Rotations to radians
        public static final double TURN_VELOCITY_CONVERSION = TURN_POSITION_CONVERSION / 60.0; // RPM to rad/s

        // Motor current limits
        public static final int DRIVE_CURRENT_LIMIT = 35; // Kraken X60 motor
        public static final int TURN_CURRENT_LIMIT = 20; // NEO motor with SparkMax
        public static final int SECONDARY_CURRENT_OFFSET = 5; // Buffer for drive motors
    }

    public static class Turn {
        // PID constants for turning motor
        public static final double P = 1.5;
        public static final double I = 0.0;
        public static final double D = 0.5;
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
