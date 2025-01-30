package frc.robot.subsystems;

import com.studica.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.module.SwerveModule;

public class DriveSubsystem extends SubsystemBase {

    private final AHRS navX;
    private final Field2d field;

    private final SwerveDriveKinematics kinematics;
    private final SwerveDriveOdometry odometry;

    private final SwerveModule frontLeftModule;
    private final SwerveModule frontRightModule;
    private final SwerveModule backLeftModule;
    private final SwerveModule backRightModule;

    private Pose2d location;

    public DriveSubsystem() {
        navX = new AHRS(AHRS.NavXComType.kUSB1);
        field = new Field2d();

        // Initialize Swerve Modules using correct constructor
        frontLeftModule = new SwerveModule(
            Constants.SwerveModules.FRONTLEFT.getDriveMotorID(),
            Constants.SwerveModules.FRONTLEFT.getTurnMotorID(),
            Constants.SwerveModules.FRONTLEFT.getEncoderOffset()
        );

        frontRightModule = new SwerveModule(
            Constants.SwerveModules.FRONTRIGHT.getDriveMotorID(),
            Constants.SwerveModules.FRONTRIGHT.getTurnMotorID(),
            Constants.SwerveModules.FRONTRIGHT.getEncoderOffset()
        );

        backLeftModule = new SwerveModule(
            Constants.SwerveModules.BACKLEFT.getDriveMotorID(),
            Constants.SwerveModules.BACKLEFT.getTurnMotorID(),
            Constants.SwerveModules.BACKLEFT.getEncoderOffset()
        );

        backRightModule = new SwerveModule(
            Constants.SwerveModules.BACKRIGHT.getDriveMotorID(),
            Constants.SwerveModules.BACKRIGHT.getTurnMotorID(),
            Constants.SwerveModules.BACKRIGHT.getEncoderOffset()
        );

        kinematics = new SwerveDriveKinematics(
            Constants.SwerveModules.FRONTLEFT.getModuleLocation(),
            Constants.SwerveModules.FRONTRIGHT.getModuleLocation(),
            Constants.SwerveModules.BACKLEFT.getModuleLocation(),
            Constants.SwerveModules.BACKRIGHT.getModuleLocation()
        );

        // Use a local variable to store the initial gyro angle instead of calling getAngle2d() directly
        Rotation2d initialGyroAngle = Rotation2d.fromDegrees(-navX.getAngle());

        odometry = new SwerveDriveOdometry(
            kinematics,
            initialGyroAngle,
            new SwerveModulePosition[]{
                frontLeftModule.getPosition(),
                frontRightModule.getPosition(),
                backLeftModule.getPosition(),
                backRightModule.getPosition()
            }
        );

        field.setRobotPose(new Pose2d());
    }

    public void resetGyroscope() {
        navX.reset();
    }

    public Pose2d getPosition() {
        return location;
    }

    public Rotation2d getAngle2d() {
        return Rotation2d.fromDegrees(-navX.getAngle());
    }

    @Override
    public void periodic() {
        location = odometry.update(
            getAngle2d(),
            new SwerveModulePosition[]{
                frontLeftModule.getPosition(),
                frontRightModule.getPosition(),
                backLeftModule.getPosition(),
                backRightModule.getPosition()
            }
        );
        field.setRobotPose(location);
    }

    public void drive(double fwd, double str, double rot, boolean fieldRelative) {
        SwerveModuleState[] states = fieldRelative
            ? kinematics.toSwerveModuleStates(
                ChassisSpeeds.fromFieldRelativeSpeeds(fwd, str, rot, getAngle2d()))
            : kinematics.toSwerveModuleStates(new ChassisSpeeds(fwd, str, rot));

        SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.Drive.MAX_SPEED);

        frontLeftModule.setDesiredState(states[0]);
        frontRightModule.setDesiredState(states[1]);
        backLeftModule.setDesiredState(states[2]);
        backRightModule.setDesiredState(states[3]);
    }
}
