package frc.robot.subsystems;

import com.studica.frc.AHRS;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.module.SwerveModule;

public class DriveSubsystem extends SubsystemBase {
    private final AHRS navX;
    private final SwerveDriveKinematics kinematics;
    private final SwerveDriveOdometry odometry;

    private final SwerveModule frontLeftModule;
    private final SwerveModule frontRightModule;
    private final SwerveModule backLeftModule;
    private final SwerveModule backRightModule;

    private Pose2d location;
    private int ticks = 0;
    private boolean driveEnabled = true;

    
    private final SlewRateLimiter fwdLimiter = new SlewRateLimiter(Constants.Drive.MAGNITUDE_SLEW_RATE);
    private final SlewRateLimiter strLimiter = new SlewRateLimiter(Constants.Drive.MAGNITUDE_SLEW_RATE);
    private final SlewRateLimiter rotLimiter = new SlewRateLimiter(Constants.Drive.ROTATIONAL_SLEW_RATE);

    public DriveSubsystem() {
        navX = new AHRS(AHRS.NavXComType.kMXP_SPI);

        frontLeftModule = new SwerveModule(ModuleConstants.FRONT_LEFT_DRIVING_CAN_ID, ModuleConstants.FRONT_LEFT_TURNING_CAN_ID, ModuleConstants.FRONT_LEFT_OFFSET);
        frontRightModule = new SwerveModule(ModuleConstants.FRONT_RIGHT_DRIVING_CAN_ID, ModuleConstants.FRONT_RIGHT_TURNING_CAN_ID, ModuleConstants.FRONT_RIGHT_OFFSET);
        backLeftModule = new SwerveModule(ModuleConstants.REAR_LEFT_DRIVING_CAN_ID, ModuleConstants.REAR_LEFT_TURNING_CAN_ID, ModuleConstants.REAR_LEFT_OFFSET);
        backRightModule = new SwerveModule(ModuleConstants.REAR_RIGHT_DRIVING_CAN_ID, ModuleConstants.REAR_RIGHT_TURNING_CAN_ID, ModuleConstants.REAR_RIGHT_OFFSET);

        kinematics = new SwerveDriveKinematics(
            ModuleConstants.FRONT_LEFT_LOCATION,
            ModuleConstants.FRONT_RIGHT_LOCATION,
            ModuleConstants.REAR_LEFT_LOCATION,
            ModuleConstants.REAR_RIGHT_LOCATION
        );

        odometry = new SwerveDriveOdometry(
            kinematics,
            getAngle2d(),
            getModulePositions()
        );

        resetGyroscope();
        resetEncoders();
    }

  
    public void resetEncoders() {
        frontLeftModule.resetEncoders();
        frontRightModule.resetEncoders();
        backLeftModule.resetEncoders();
        backRightModule.resetEncoders();
    }


    public void resetGyroscope() {
        navX.reset();
    }

   
    public Rotation2d getAngle2d() {
        return Rotation2d.fromDegrees(-navX.getYaw());
    }

   
    public double getHeading() {
        return getAngle2d().getDegrees();
    }


    public AHRS getNavX() {
        return navX;
    }

    private SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[]{
            frontLeftModule.getPosition(),
            frontRightModule.getPosition(),
            backLeftModule.getPosition(),
            backRightModule.getPosition()
        };
    }

    @Override
    public void periodic() {
        ticks++;
        if (ticks > 5) { 
            updateDashboard();
            ticks = 0;
        }

        if (driveEnabled) {
            updateOdometry();
        }
    }

   
    private void updateOdometry() {
        location = odometry.update(getAngle2d(), getModulePositions());
    }

    public Pose2d getPosition() {
    return location;
    }

   
    private void updateDashboard() {
        SmartDashboard.putNumber("Odometry X", location.getX());
        SmartDashboard.putNumber("Odometry Y", location.getY());
        SmartDashboard.putNumber("Gyro Heading", getAngle2d().getDegrees());
        SmartDashboard.putBoolean("Drive Enabled", driveEnabled);
    }

    public void drive(double fwd, double str, double rot, boolean fieldRelative) {
        fwd = fwdLimiter.calculate(fwd);
        str = strLimiter.calculate(str);
        rot = rotLimiter.calculate(rot);

        SwerveModuleState[] states = kinematics.toSwerveModuleStates(
            fieldRelative 
                ? ChassisSpeeds.fromFieldRelativeSpeeds(fwd, str, rot, getAngle2d())
                : new ChassisSpeeds(fwd, str, rot)
        );

        SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.Drive.MAX_SPEED);

        frontLeftModule.setDesiredState(states[0]);
        frontRightModule.setDesiredState(states[1]);
        backLeftModule.setDesiredState(states[2]);
        backRightModule.setDesiredState(states[3]);
    }

    

    @Override
    public void simulationPeriodic() {}
}
