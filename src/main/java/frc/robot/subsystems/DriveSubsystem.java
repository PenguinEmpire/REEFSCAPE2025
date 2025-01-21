package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.drivetrain.SwerveModule;

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
        // Initialize components
        navX = new AHRS(SerialPort.Port.kUSB);
        field = new Field2d();

        // Initialize Swerve Modules
        frontLeftModule = createSwerveModule(Constants.SwerveModules.FRONTLEFT, "FrontLeft");
        frontRightModule = createSwerveModule(Constants.SwerveModules.FRONTRIGHT, "FrontRight");
        backLeftModule = createSwerveModule(Constants.SwerveModules.BACKLEFT, "BackLeft");
        backRightModule = createSwerveModule(Constants.SwerveModules.BACKRIGHT, "BackRight");

        // Initialize kinematics and odometry
        kinematics = new SwerveDriveKinematics(
            Constants.SwerveModules.FRONTLEFT.getModuleLocation(),
            Constants.SwerveModules.FRONTRIGHT.getModuleLocation(),
            Constants.SwerveModules.BACKLEFT.getModuleLocation(),
            Constants.SwerveModules.BACKRIGHT.getModuleLocation()
        );

        odometry = new SwerveDriveOdometry(
            kinematics,
            getAngle2d(),
            new SwerveModulePosition[] {
                frontLeftModule.getPosition(),
                frontRightModule.getPosition(),
                backLeftModule.getPosition(),
                backRightModule.getPosition()
            }
        );

        // Publish data to SmartDashboard
        SmartDashboard.putData("Field", field);
        resetGyroscope();
    }

    private SwerveModule createSwerveModule(Constants.SwerveModules module, String name) {
        return new SwerveModule(
            name,
            module.getDriveMotorID(),
            module.getTurnMotorID(),
            module.getEncoderPort(),
            module.getModuleLocation(),
            module.getEncoderOffset()
        );
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

    public void periodic() {
        // Update odometry periodically
        location = odometry.update(
            getAngle2d(),
            new SwerveModulePosition[] {
                frontLeftModule.getPosition(),
                frontRightModule.getPosition(),
                backLeftModule.getPosition(),
                backRightModule.getPosition()
            }
        );

        // Update field visualization
        field.setRobotPose(location);
    }

    public void drive(double fwd, double str, double rot, boolean fieldRelative) {
        SwerveModuleState[] states;
        if (fieldRelative) {
            states = kinematics.toSwerveModuleStates(
                ChassisSpeeds.fromFieldRelativeSpeeds(fwd, str, rot, getAngle2d())
            );
        } else {
            states = kinematics.toSwerveModuleStates(new ChassisSpeeds(fwd, str, rot));
        }

        SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.Drive.MAX_SPEED);

        frontLeftModule.setTargetState(states[0]);
        frontRightModule.setTargetState(states[1]);
        backLeftModule.setTargetState(states[2]);
        backRightModule.setTargetState(states[3]);
    }
}
