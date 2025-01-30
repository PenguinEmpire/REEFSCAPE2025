package frc.robot.utils;

public class SwerveUtils {

    /**
     * Steps a value towards a target with a specified step size.
     * @param _current The current or starting value.  Can be positive or negative.
     * @param _target The target value the algorithm will step towards.  Can be positive or negative.
     * @param _stepsize The maximum step size that can be taken.
     * @return The new value for {@code _current} after performing the specified step towards the specified target.
     */
    public static double StepTowards(double _current, double _target, double _stepsize) {
        if (Math.abs(_current - _target) <= _stepsize) {
            return _target;
        }
        else if (_target < _current) {
            return _current - _stepsize;
        }
        else {
            return _current + _stepsize;
        }
    }

    /**
     * Steps a value (angle) towards a target (angle) taking the shortest path with a specified step size.
     * @param _current The current or starting angle (in radians).  Can lie outside the 0 to 2*PI range.
     * @param _target The target angle (in radians) the algorithm will step towards.  Can lie outside the 0 to 2*PI range.
     * @param _stepsize The maximum step size that can be taken (in radians).
     * @return The new angle (in radians) for {@code _current} after performing the specified step towards the specified target.
     * the value wil be between 0 and 2PI
     */
    public static double StepTowardsCircular(double _current, double _target, double _stepsize) {
        _current = WrapAngle(_current);
        _target = WrapAngle(_target);

        double stepDirection = Math.signum(_target - _current);
        double difference = Math.abs(_current - _target);
        
        if (difference <= _stepsize) {
            return _target;
        }
        else if (difference > Math.PI) { //does the system need to wrap over eventually?
            //handle the special case where you can reach the target in one step while also wrapping
            if (_current + 2*Math.PI - _target < _stepsize || _target + 2*Math.PI - _current < _stepsize) {
                return _target;
            }
            else {
                return WrapAngle(_current - stepDirection * _stepsize); //this will handle wrapping gracefully
            }

        }
        else {
            return _current + stepDirection * _stepsize;
        }
    }

    /**
     * Finds the (unsigned) minimum difference between two angles including calculating across 0.
     * @param _angleA An angle (in radians).
     * @param _angleB An angle (in radians).
     * @return The (unsigned) minimum difference between the two angles (in radians).
     */
    public static double AngleDifference(double _angleA, double _angleB) {
        double difference = Math.abs(_angleA - _angleB);
        return difference > Math.PI? (2 * Math.PI) - difference : difference;
    }

    /**
     * Wraps an angle until it lies within the range from 0 to 2*PI (exclusive).
     * @param _angle The angle (in radians) to wrap.  Can be positive or negative and can lie multiple wraps outside the output range.
     * @return An angle (in radians) from 0 and 2*PI (exclusive).
     */
    public static double WrapAngle(double _angle) {
        double twoPi = 2*Math.PI;

        if (_angle == twoPi) { 
            return 0.0;
        }
        else if (_angle > twoPi) {
            return _angle - twoPi*Math.floor(_angle / twoPi);
        }
        else if (_angle < 0.0) {
            return _angle + twoPi*(Math.floor((-_angle) / twoPi)+1);
        }
        else {
            return _angle;
        }
    }

    // New methods for Kraken X60 and NEO conversion

    /**
     * Converts a velocity command to the Kraken X60 motor's native units.
     * @param velocity The target velocity in meters per second.
     * @param wheelDiameter The diameter of the wheel in meters.
     * @param gearReduction The gear reduction ratio.
     * @return The velocity in Kraken X60 motor encoder units.
     */
    public static double velocityToMotorUnits(double velocity, double wheelDiameter, double gearReduction) {
        double wheelCircumference = Math.PI * wheelDiameter;  // Circumference of the wheel (in meters)
        double rotationsPerSecond = velocity / wheelCircumference;  // Speed in wheel rotations per second
        double motorUnitsPerRevolution = 2048;  // Kraken X60 has 2048 ticks per full rotation (native encoder units)
        double motorSpeedUnits = rotationsPerSecond * motorUnitsPerRevolution * gearReduction;  // Apply gear reduction
        return motorSpeedUnits;
    }

    /**
     * Converts an angle (in radians) to a steering motor position.
     * Ensures the angle stays within the 0 to 2*PI range for smooth control.
     * @param angle The target angle in radians.
     * @return The target angle wrapped in the 0 to 2*PI range.
     */
    public static double angleToMotorPosition(double angle) {
        return WrapAngle(angle);  // Ensures the angle is within the 0 to 2*PI range
    }

    /**
     * Converts an angular velocity command to NEO steering motor's native units.
     * This is useful when setting up rotational velocity commands for swerve drive.
     * @param angularVelocity The target angular velocity in radians per second.
     * @param wheelBase The distance between the wheels.
     * @param gearReduction The gear reduction ratio for the steering motor.
     * @return The angular velocity in NEO steering motor encoder units.
     */
    public static double angularVelocityToMotorUnits(double angularVelocity, double wheelBase, double gearReduction) {
        double motorUnitsPerRevolution = 2048;  // NEO has 2048 ticks per full rotation (native encoder units)
        double rotationsPerSecond = angularVelocity / (2 * Math.PI);  // Convert radians per second to rotations per second
        return rotationsPerSecond * motorUnitsPerRevolution * gearReduction;
    }
}
