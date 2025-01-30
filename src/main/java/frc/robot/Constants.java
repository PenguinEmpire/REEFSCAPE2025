package frc.robot;

import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.math.geometry.Translation2d;

public final class Constants {
    public static class Drive {
        public static final double WHEEL_DIAMETER = 0.1016;
        public static final double DRIVE_REDUCTION = 8.33;
        public static final double DRIVE_CONVERSION = WHEEL_DIAMETER * Math.PI / DRIVE_REDUCTION;
        public static final double MAX_SPEED = 3.0;
        public static final double MAX_ANGULAR_SPEED = Math.PI;

        public static final double KRAKEN_TICKS_PER_ROTATION = 2048.0;
        public static final double KRAKEN_POSITION_CONVERSION = DRIVE_CONVERSION / KRAKEN_TICKS_PER_ROTATION;
        public static final double KRAKEN_VELOCITY_CONVERSION = KRAKEN_POSITION_CONVERSION * 10.0;

        public static final double TURN_GEAR_RATIO = 12.0;
        public static final double TURN_POSITION_CONVERSION = 2 * Math.PI / TURN_GEAR_RATIO;
        public static final double TURN_VELOCITY_CONVERSION = TURN_POSITION_CONVERSION / 60.0;

        public static final int KRAKEN_CURRENT_LIMIT = 35;
        public static final int TURN_CURRENT_LIMIT = 20;

        public static final double KRAKEN_P = 0.05;
        public static final double KRAKEN_I = 0.0;
        public static final double KRAKEN_D = 0.0;
        public static final double KRAKEN_KV = 0.12;
        public static final double KRAKEN_KS = 0.24;

        public static final double TURN_P = 1.5;
        public static final double TURN_I = 0.0;
        public static final double TURN_D = 0.5;

        public static final double TURN_ENCODER_POSITION_PID_MIN_INPUT = 0.0;
        public static final double TURN_ENCODER_POSITION_PID_MAX_INPUT = 2 * Math.PI;

        public static final InvertedValue KRAKEN_INVERTED = InvertedValue.Clockwise_Positive;
        public static final boolean TURN_MOTOR_INVERTED = true;
    }

    public static class Turn {
        public static final double P = Drive.TURN_P;
        public static final double I = Drive.TURN_I;
        public static final double D = Drive.TURN_D;
    }

    public static class Shooter {
        public static final int MOTOR_ID = 10;  
        public static final int LIMIT_SWITCH_CHANNEL = 2;
        public static final double FORWARD_POWER = 0.8;
        public static final double REVERSE_POWER = -0.8;
        public static final double STOP_POWER = 0.0;
    }

    public static class Intake {
        public static final int HORIZONTAL_ROLLER_MOTOR_ID = 11;
        public static final int LEFT_VERTICAL_ROLLER_MOTOR_ID = 12;
        public static final int RIGHT_VERTICAL_ROLLER_MOTOR_ID = 13;
        public static final int ROTATION_MOTOR_ID = 14;

        public static final int BORE_ENCODER_CHANNEL = 3;

        public static final double DEFAULT_ROLLER_POWER = 0.75;
        public static final int MOTOR_CURRENT_LIMIT = 30;

        public static final double ROTATION_P = 0.05;
        public static final double ROTATION_I = 0.0;
        public static final double ROTATION_D = 0.0;

        public static final double ROTATION_MAX_ANGLE = 90.0;
        public static final double ROTATION_TOLERANCE = 2.0;
    }

    public static class Elevator {
        public static final int LEFT_MOTOR_ID = 15;
        public static final int RIGHT_MOTOR_ID = 16;
        public static final int LIMIT_SWITCH_CHANNEL = 4;

        public static final double KP = 0.1;
        public static final double KI = 0.0;
        public static final double KD = 0.0;

        public static final double ENCODER_CONVERSION_FACTOR = 0.001;
        public static final double INTAKE_POSITION = 0.2;
        public static final double REEF_LEVEL_1 = 0.5;
        public static final double REEF_LEVEL_2 = 1.0;
        public static final double REEF_LEVEL_3 = 1.5;
        public static final double REEF_LEVEL_4 = 2.0;
        
        public static final double TOLERANCE = 0.05;
    }

    public static class AlgaeHolder {
        public static final int PIVOT_MOTOR_ID = 17;
        public static final int ROLLING_MOTOR_ID = 18;
        public static final int PIVOT_ENCODER_CHANNEL = 5;
        public static final int LIMIT_SWITCH_CHANNEL = 6;

        public static final double PIVOT_P = 0.1;
        public static final double PIVOT_I = 0.0;
        public static final double PIVOT_D = 0.0;

        public static final double ROLLING_POWER = 0.5;
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

        public int getEncoderPort() { return encoderPort; }
        public int getDriveMotorID() { return driveMotorID; }
        public int getTurnMotorID() { return turnMotorID; }
        public Translation2d getModuleLocation() { return moduleLocation; }
        public double getEncoderOffset() { return encoderOffset; }
    }
}
