package frc.robot;

import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.math.geometry.Translation2d;

public final class Constants {
    public static class Drive {
        public static final double WHEEL_DIAMETER = 0.1016; // 4 inches
        public static final double DRIVE_REDUCTION = 8.33;  // Same as last year (Swerve X)
        public static final double DRIVE_CONVERSION = WHEEL_DIAMETER * Math.PI / DRIVE_REDUCTION;
        public static final double MAX_SPEED = 4.7; 
        public static final double MAX_ANGULAR_SPEED = 2 * Math.PI;

        // Slew Rates (to prevent sharp accelerations)
        public static final double DIRECTION_SLEW_RATE = 1.2; // radians per second
        public static final double MAGNITUDE_SLEW_RATE = 1.8; // percent per second
        public static final double ROTATIONAL_SLEW_RATE = 2.0; // percent per second

        // Kraken X60 Encoder Values
        public static final double KRAKEN_TICKS_PER_ROTATION = 2048.0;
        public static final double KRAKEN_POSITION_CONVERSION = DRIVE_CONVERSION / KRAKEN_TICKS_PER_ROTATION;
        public static final double KRAKEN_VELOCITY_CONVERSION = KRAKEN_POSITION_CONVERSION * 10.0;

        // Turn Motor Configurations
        public static final double TURN_GEAR_RATIO = 12.0;
        public static final double TURN_POSITION_CONVERSION = 2 * Math.PI / TURN_GEAR_RATIO;
        public static final double TURN_VELOCITY_CONVERSION = TURN_POSITION_CONVERSION / 60.0;

        // Current Limits
        public static final int KRAKEN_CURRENT_LIMIT = 40; // Updated for Kraken X60
        public static final int TURN_CURRENT_LIMIT = 30;

        // Kraken X60 PID Values (Initial Estimates - Tune on actual bot)
        public static final double KRAKEN_P = 0.08;  
        public static final double KRAKEN_I = 0.0001;  
        public static final double KRAKEN_D = 0.01; 
        public static final double KRAKEN_KV = 0.12;
        public static final double KRAKEN_KS = 0.24;

        // Turning PID
        public static final double TURN_P = 1.5;
        public static final double TURN_I = 0.0;
        public static final double TURN_D = 0.5;

        // Turning Encoder Range
        public static final double TURN_ENCODER_POSITION_PID_MIN_INPUT = 0.0;
        public static final double TURN_ENCODER_POSITION_PID_MAX_INPUT = 2 * Math.PI;

        // Motor Directions
        public static final InvertedValue KRAKEN_INVERTED = InvertedValue.Clockwise_Positive;
        public static final boolean TURN_MOTOR_INVERTED = true;
    }

    public static class ModuleConstants {
        // Motor CAN IDs
        public static final int FRONT_LEFT_TURNING_CAN_ID = 7;
        public static final int FRONT_RIGHT_TURNING_CAN_ID = 26;
        public static final int REAR_LEFT_TURNING_CAN_ID = 8;
        public static final int REAR_RIGHT_TURNING_CAN_ID = 12;

        public static final int FRONT_LEFT_DRIVING_CAN_ID = 2;
        public static final int FRONT_RIGHT_DRIVING_CAN_ID = 1;
        public static final int REAR_LEFT_DRIVING_CAN_ID = 3;
        public static final int REAR_RIGHT_DRIVING_CAN_ID = 4;
        
        public static final boolean TURNING_MOTOR_INVERTED = true;

        // Module locations (relative to the robot center)
        public static final Translation2d FRONT_LEFT_LOCATION = new Translation2d(0.381, 0.381);
        public static final Translation2d FRONT_RIGHT_LOCATION = new Translation2d(0.381, -0.381);
        public static final Translation2d REAR_LEFT_LOCATION = new Translation2d(-0.381, 0.381);
        public static final Translation2d REAR_RIGHT_LOCATION = new Translation2d(-0.381, -0.381);

        // Angular offsets for swerve modules (last year's values)
        public static final double FRONT_LEFT_OFFSET = -(0.0431 + 0.39216 + Math.PI - (2 * Math.PI));
        public static final double FRONT_RIGHT_OFFSET = -(1.57568);
        public static final double REAR_LEFT_OFFSET = -(-0.97338 + 6.2831);
        public static final double REAR_RIGHT_OFFSET = -(0.278867);
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
        public static final int LEFT_VERTICAL_ROLLER_MOTOR_ID = 19;
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
}
