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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ModuleConstants;
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

        // Initialize Swerve Modules
        frontLeftModule = new SwerveModule(
            ModuleConstants.FRONT_LEFT_DRIVING_CAN_ID,
            ModuleConstants.FRONT_LEFT_TURNING_CAN_ID,
            ModuleConstants.FRONT_LEFT_OFFSET
        );

        frontRightModule = new SwerveModule(
            ModuleConstants.FRONT_RIGHT_DRIVING_CAN_ID,
            ModuleConstants.FRONT_RIGHT_TURNING_CAN_ID,
            ModuleConstants.FRONT_RIGHT_OFFSET
        );

        backLeftModule = new SwerveModule(
            ModuleConstants.REAR_LEFT_DRIVING_CAN_ID,
            ModuleConstants.REAR_LEFT_TURNING_CAN_ID,
            ModuleConstants.REAR_LEFT_OFFSET
        );

        backRightModule = new SwerveModule(
            ModuleConstants.REAR_RIGHT_DRIVING_CAN_ID,
            ModuleConstants.REAR_RIGHT_TURNING_CAN_ID,
            ModuleConstants.REAR_RIGHT_OFFSET
        );

        kinematics = new SwerveDriveKinematics(
            ModuleConstants.FRONT_LEFT_LOCATION,
            ModuleConstants.FRONT_RIGHT_LOCATION,
            ModuleConstants.REAR_LEFT_LOCATION,
            ModuleConstants.REAR_RIGHT_LOCATION
        );

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


    public AHRS getNavX() {
        return navX;
    }

    public void resetGyroscope() {
        navX.reset();
    }

    public double getHeading() {
        return Rotation2d.fromDegrees(-navX.getYaw()).getDegrees();
    }

    public Pose2d getPosition() {
        return location;
    }

    public void resetOdometry(Pose2d pose) {
        odometry.resetPosition(
            Rotation2d.fromDegrees(-navX.getYaw()),
            new SwerveModulePosition[]{
                frontLeftModule.getPosition(),
                frontRightModule.getPosition(),
                backLeftModule.getPosition(),
                backRightModule.getPosition()
            },
            pose);
    }

    @Override
    public void periodic() {
        location = odometry.update(
            Rotation2d.fromDegrees(-navX.getYaw()),
            new SwerveModulePosition[]{
                frontLeftModule.getPosition(),
                frontRightModule.getPosition(),
                backLeftModule.getPosition(),
                backRightModule.getPosition()
            }
        );

        field.setRobotPose(location);

        SmartDashboard.putNumber("Odometry X", location.getX());
        SmartDashboard.putNumber("Odometry Y", location.getY());
        SmartDashboard.putNumber("Gyro Yaw", navX.getYaw());
        SmartDashboard.putNumber("Gyro Angle", navX.getAngle());
        SmartDashboard.putNumber("Gyro Heading", getHeading());
    }

    public void drive(double fwd, double str, double rot, boolean fieldRelative) {
        SwerveModuleState[] states = fieldRelative
            ? kinematics.toSwerveModuleStates(
                ChassisSpeeds.fromFieldRelativeSpeeds(fwd, str, rot, Rotation2d.fromDegrees(-navX.getAngle())))
            : kinematics.toSwerveModuleStates(new ChassisSpeeds(fwd, str, rot));

        SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.Drive.MAX_SPEED);

        frontLeftModule.setDesiredState(states[0]);
        frontRightModule.setDesiredState(states[1]);
        backLeftModule.setDesiredState(states[2]);
        backRightModule.setDesiredState(states[3]);
    }

    public void resetEncoders() {
        frontLeftModule.resetEncoders();
        backLeftModule.resetEncoders();
        frontRightModule.resetEncoders();
        backRightModule.resetEncoders();
    }
}
