package frc.robot.commands.autonomous;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

public class MoveCommandOdometry extends Command {
    private final DriveSubsystem m_subsystem;
    private final Pose2d m_targetPosition;
    private final boolean m_keepVelocity;
    private final double m_maxSpeed;

    private final PIDController xPID;
    private final PIDController yPID;
    private final PIDController turnPID;

    private int m_ticks;

    public MoveCommandOdometry(DriveSubsystem subsystem, Pose2d targetPosition, boolean keepVelocity, double maxSpeed) {
        m_subsystem = subsystem;
        m_targetPosition = targetPosition;
        m_keepVelocity = keepVelocity;
        m_maxSpeed = maxSpeed;

        xPID = new PIDController(3, 0.001, 0);
        yPID = new PIDController(3, 0.001, 0);
        turnPID = new PIDController(0.04, 0, 0);

        addRequirements(m_subsystem);
    }

    @Override
    public void initialize() {
        xPID.reset();
        yPID.reset();
        turnPID.reset();

        xPID.setTolerance(0.02);
        yPID.setTolerance(0.02);
        turnPID.setTolerance(1);
        turnPID.enableContinuousInput(-180, 180);
        m_ticks = 0;
    }

    public double dclamp(double x, double r) {
        return x > r ? r : (x < -r ? -r : x);
    }

    @Override
    public void execute() {
        m_ticks++;
        Pose2d currentPose = m_subsystem.getPose();
        double xValue = xPID.calculate(currentPose.getTranslation().getX(), m_targetPosition.getTranslation().getX());
        double yValue = yPID.calculate(currentPose.getTranslation().getY(), m_targetPosition.getTranslation().getY());
        double turnValue = -turnPID.calculate(currentPose.getRotation().getDegrees(),
                m_targetPosition.getRotation().getDegrees());
        double tickLength = 20;
        if (m_ticks > tickLength)
            m_ticks = (int) tickLength;
        double maxDrive = m_maxSpeed * (m_ticks / tickLength);
        double maxRot = 0.4 * (m_ticks / tickLength);
        m_subsystem.drive(dclamp(xValue, maxDrive), dclamp(yValue, maxDrive), dclamp(turnValue, maxRot), true, false);
    }

    @Override
    public void end(boolean interrupted) {
        m_subsystem.drive(0, 0, 0, false, true);
    }

    @Override
    public boolean isFinished() {
        double posError = m_keepVelocity ? 0.02 : 0.009;
        double velError = m_keepVelocity ? 9999 : 0.1;
    
        // Compute position error manually
        double xPositionError = xPID.getSetpoint() - m_subsystem.getPose().getX();
        double yPositionError = yPID.getSetpoint() - m_subsystem.getPose().getY();
    
        // Compute velocity error manually using derivative of error
        double xVelocityError = xPID.getSetpoint() - (m_subsystem.getPose().getX() + xPID.getP() * xPositionError);
        double yVelocityError = yPID.getSetpoint() - (m_subsystem.getPose().getY() + yPID.getP() * yPositionError);
    
        return Math.abs(xPositionError) < posError && Math.abs(xVelocityError) < velError &&
               Math.abs(yPositionError) < posError && Math.abs(yVelocityError) < velError;
    }
    
}
    