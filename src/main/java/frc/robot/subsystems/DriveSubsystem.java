package frc.robot.subsystems;

import com.studica.frc.AHRS;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.module.SwerveModule;
import frc.robot.utils.SwerveUtils;

public class DriveSubsystem extends SubsystemBase {

    private static final double DEADBAND = 0.05; 

    private final AHRS navX = new AHRS(AHRS.NavXComType.kMXP_SPI);
    private final PIDController rotationPID = new PIDController(0.015, 0, 0.0001);

    private final SwerveModule m_frontLeft = new SwerveModule(
            Constants.ModuleConstants.FRONT_LEFT_DRIVING_CAN_ID,
            Constants.ModuleConstants.FRONT_LEFT_TURNING_CAN_ID,
            Constants.ModuleConstants.FRONT_LEFT_OFFSET);

    private final SwerveModule m_frontRight = new SwerveModule(
            Constants.ModuleConstants.FRONT_RIGHT_DRIVING_CAN_ID,
            Constants.ModuleConstants.FRONT_RIGHT_TURNING_CAN_ID,
            Constants.ModuleConstants.FRONT_RIGHT_OFFSET);

    private final SwerveModule m_rearLeft = new SwerveModule(
            Constants.ModuleConstants.REAR_LEFT_DRIVING_CAN_ID,
            Constants.ModuleConstants.REAR_LEFT_TURNING_CAN_ID,
            Constants.ModuleConstants.REAR_LEFT_OFFSET);

    private final SwerveModule m_rearRight = new SwerveModule(
            Constants.ModuleConstants.REAR_RIGHT_DRIVING_CAN_ID,
            Constants.ModuleConstants.REAR_RIGHT_TURNING_CAN_ID,
            Constants.ModuleConstants.REAR_RIGHT_OFFSET);

    private double m_currentRotation = 0.0;
    private double m_currentTranslationDir = 0.0;
    private double m_currentTranslationMag = 0.0;

    private final SlewRateLimiter m_magLimiter = new SlewRateLimiter(Constants.Drive.MAGNITUDE_SLEW_RATE);
    private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(Constants.Drive.ROTATIONAL_SLEW_RATE);
    private double m_prevTime = WPIUtilJNI.now() * 1e-6;

    private final SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(
            Constants.Drive.KINEMATICS,
            Rotation2d.fromDegrees(-navX.getYaw()),
            new SwerveModulePosition[]{
                    m_frontLeft.getPosition(),
                    m_frontRight.getPosition(),
                    m_rearLeft.getPosition(),
                    m_rearRight.getPosition()
            });

    public DriveSubsystem() {
        rotationPID.reset();
    }

    @Override
    public void periodic() {
        m_odometry.update(
                Rotation2d.fromDegrees(-navX.getYaw()),
                new SwerveModulePosition[]{
                        m_frontLeft.getPosition(),
                        m_frontRight.getPosition(),
                        m_rearLeft.getPosition(),
                        m_rearRight.getPosition()
                });

        SmartDashboard.putNumber("Odometry X", m_odometry.getPoseMeters().getX());
        SmartDashboard.putNumber("Odometry Y", m_odometry.getPoseMeters().getY());
        SmartDashboard.putNumber("Gyro Yaw", navX.getYaw());
        SmartDashboard.putNumber("Gyro Heading", getHeading());
    }

    public Pose2d getPose() {
        return m_odometry.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        m_odometry.resetPosition(
                Rotation2d.fromDegrees(-navX.getYaw()),
                new SwerveModulePosition[]{
                        m_frontLeft.getPosition(),
                        m_frontRight.getPosition(),
                        m_rearLeft.getPosition(),
                        m_rearRight.getPosition()
                },
                pose);
    }

    public void setChassisSpeeds(ChassisSpeeds speeds) {
        speeds.omegaRadiansPerSecond *= -1;

        SwerveModuleState[] swerveModuleStates = Constants.Drive.KINEMATICS.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Drive.MAX_SPEED);
        setModuleStates(swerveModuleStates);
    }

    public ChassisSpeeds getRobotRelativeSpeeds() {
        return Constants.Drive.KINEMATICS.toChassisSpeeds(
                m_frontLeft.getState(),
                m_frontRight.getState(),
                m_rearLeft.getState(),
                m_rearRight.getState());
    }

    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative, boolean rateLimit) {
        //  Apply deadband to prevent small unintended movements
        xSpeed = Math.abs(xSpeed) > DEADBAND ? xSpeed : 0;
        ySpeed = Math.abs(ySpeed) > DEADBAND ? ySpeed : 0;
        rot = Math.abs(rot) > DEADBAND ? rot : 0;

        double xSpeedCommanded;
        double ySpeedCommanded;

        if (rateLimit) {
            double inputTranslationDir = Math.atan2(ySpeed, xSpeed);
            double inputTranslationMag = Math.hypot(xSpeed, ySpeed);

            double directionSlewRate = (m_currentTranslationMag != 0.0)
                    ? Math.abs(Constants.Drive.DIRECTION_SLEW_RATE / m_currentTranslationMag)
                    : 500.0;

            double currentTime = WPIUtilJNI.now() * 1e-6;
            double elapsedTime = currentTime - m_prevTime;
            double angleDif = SwerveUtils.AngleDifference(inputTranslationDir, m_currentTranslationDir);

            if (angleDif < 0.45 * Math.PI) {
                m_currentTranslationDir = SwerveUtils.StepTowardsCircular(m_currentTranslationDir, inputTranslationDir, directionSlewRate * elapsedTime);
                m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
            } else {
                m_currentTranslationDir = SwerveUtils.WrapAngle(m_currentTranslationDir + Math.PI);
                m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
            }

            m_prevTime = currentTime;

            xSpeedCommanded = m_currentTranslationMag * Math.cos(m_currentTranslationDir);
            ySpeedCommanded = m_currentTranslationMag * Math.sin(m_currentTranslationDir);
            m_currentRotation = m_rotLimiter.calculate(rot);
        } else {
            xSpeedCommanded = xSpeed;
            ySpeedCommanded = ySpeed;
            m_currentRotation = rot;
        }

        double xSpeedDelivered = xSpeedCommanded * Constants.Drive.MAX_SPEED;
        double ySpeedDelivered = ySpeedCommanded * Constants.Drive.MAX_SPEED;
        double rotDelivered = m_currentRotation * Constants.Drive.MAX_ANGULAR_SPEED;

        var swerveModuleStates = Constants.Drive.KINEMATICS.toSwerveModuleStates(
                fieldRelative
                        ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered, Rotation2d.fromDegrees(-navX.getAngle()))
                        : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));

        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Drive.MAX_SPEED);
        setModuleStates(swerveModuleStates);
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Drive.MAX_SPEED);
        m_frontLeft.setDesiredState(desiredStates[0]);
        m_frontRight.setDesiredState(desiredStates[1]);
        m_rearLeft.setDesiredState(desiredStates[2]);
        m_rearRight.setDesiredState(desiredStates[3]);
    }

    public double getHeading() {
        return Rotation2d.fromDegrees(-navX.getYaw()).getDegrees();
    }

    public void zeroHeading() {
        navX.reset();
    }
}
