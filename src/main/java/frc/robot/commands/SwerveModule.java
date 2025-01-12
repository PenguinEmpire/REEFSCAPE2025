@ -1,188 +1,189 @@
package frc.robot.drivetrain;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogEncoder;
import frc.robot.Constants.Drive;
import frc.robot.Constants.Turn;

public class SwerveModule {
    //boilerplate fields
    public final String name;
    private final int driveMotorID;
    private final int turnMotorID;
    private final int analogEncoderPort;
    private final Translation2d locationOffset;
    //analog offset not final because we fuck with it a lot
    private double analogOffset;

    //motors
    private CANSparkMax driveMotor;
    private CANSparkMax turnMotor;

    //sensor input
    private AnalogEncoder analogEncoder;
    private SparkMaxRelativeEncoder driveEncoder;
    private SparkMaxRelativeEncoder turnEncoder;

    //pid controllers
    private SparkMaxPIDController drivePIDController;
    private SparkMaxPIDController turnPIDController;
    
    //module state
    private Translation2d m_position;
    //radians
    private double encoderAngle = 0;
    //meters/s
    private double currentVelocity = 0;

    private double targetVelocity = 0;
    private double targetAngle = 0;

    public SwerveModule(String name, int driveID, int turnID, int analogEncoderPort, Translation2d locationOffset, double analogOffset) {
        //boilerplate initialization
        System.out.println(name + " Initialized");
        this.name = name;
        this.driveMotorID = driveID;
        this.turnMotorID = turnID;
        this.analogEncoderPort = analogEncoderPort;
        this.locationOffset = locationOffset;
        this.analogOffset = analogOffset;


        //create motor objects
        driveMotor = new CANSparkMax(driveMotorID, MotorType.kBrushless);
        turnMotor = new CANSparkMax(turnMotorID, MotorType.kBrushless);

        //create analog absolute encoder object
        analogEncoder = new AnalogEncoder(this.analogEncoderPort);
        
        //create hall effect sensor encoder objects
        //casting because its type is RelativeEncoder, but returns SparkMaxRelativeEncoder for some reason
        driveEncoder = (SparkMaxRelativeEncoder) driveMotor.getEncoder();
        turnEncoder = (SparkMaxRelativeEncoder) turnMotor.getEncoder();

        //create pid controller objects from internal sparkmax pid controller
        drivePIDController = driveMotor.getPIDController();
        turnPIDController = turnMotor.getPIDController();
        
        

        // configure analog absolute and CAN relative encoders
        analogEncoder.setDistancePerRotation(Drive.DISTANCE_PER_ROT);
        turnEncoder.setPositionConversionFactor(Drive.ROT_POSITION_CONVERSION_FACTOR);

        //run wheel realignment method
        realignWheel();

        //current limits for drive and current motors
        driveMotor.setSmartCurrentLimit(Drive.DRIVE_CURRENT_LIMIT);
        driveMotor.setSecondaryCurrentLimit(Drive.DRIVE_CURRENT_LIMIT + Drive.SECONDARY_CURRENT_OFFSET);
        turnMotor.setSmartCurrentLimit(Drive.TURN_CURRENT_LIMIT);
        turnMotor.setSecondaryCurrentLimit(Drive.TURN_CURRENT_LIMIT + Drive.SECONDARY_CURRENT_OFFSET);

        //zero relative encoder position
            //sds - https://github.com/FRCTeam2910/Common/blob/a240f39c9c3d3ae9f1e74c11cffe07e314c743bd/robot/src/main/java/org/frcteam2910/common/robot/drivers/Mk2SwerveModuleBuilder.java#L305
            // encoder.setPositionConversionFactor(wheelDiameter * Math.PI / reduction);
            // encoder.setVelocityConversionFactor(wheelDiameter * Math.PI / reduction * (1.0 / 60.0)); // RPM to units per second
        driveEncoder.setPosition(0);
        driveEncoder.setPositionConversionFactor(Drive.DRIVE_CONVERSION);
        driveEncoder.setVelocityConversionFactor(Drive.DRIVE_CONVERSION / 60);

        //setup set p,i,d constants for turn motors
        turnPIDController.setP(Turn.P);
        turnPIDController.setI(Turn.I);
        turnPIDController.setD(Turn.D);

        readHardware();

    }

    public void readHardware() {
        encoderAngle = getAnalogEncoderAngle();
        // in meters/s
        currentVelocity = driveEncoder.getVelocity();
    }

    public void moveTowardsTarget() {
        double targetSpeed = targetVelocity;
        double tAngle = targetAngle;

        double currentAngle = turnEncoder.getPosition();

        // Calculate the current angle in the range [0, 2pi)
        double currentAngleMod = currentAngle % (2.0 * Math.PI);
        if (currentAngleMod < 0.0) {
            currentAngleMod += 2.0 * Math.PI;
        }

        // Figure out target to send to Spark MAX because the encoder is continuous
        double newTarget = tAngle + currentAngle - currentAngleMod;
        if (tAngle - currentAngleMod > Math.PI) {
            newTarget -= 2.0 * Math.PI;
        } else if (tAngle - currentAngleMod < -Math.PI) {
            newTarget += 2.0 * Math.PI;
        }

        driveMotor.set(targetSpeed);
        turnPIDController.setReference(newTarget, ControlType.kPosition);
    }

    public void setTargetState(SwerveModuleState state) {
        SwerveModuleState stateOptimized = SwerveModuleState.optimize(state, Rotation2d.fromRadians(turnEncoder.getPosition()));
        targetVelocity = stateOptimized.speedMetersPerSecond;
        targetAngle = stateOptimized.angle.getRadians();
    }

    public void realignWheel() {
        turnEncoder.setPosition(getAnalogEncoderAngle());
    }

    public void updateAnalogOffset(double offset) {
        this.analogOffset = offset;
    }

    public double getAnalogOffset() {
        return this.analogOffset;
    }

    //in radians
    public double getAnalogEncoderAngle() {
        double angle = (1.0 - analogEncoder.getAbsolutePosition()) * 2 * Math.PI;
        angle += analogOffset;
        return angle;
    }
    //wrapped
    public double getRawAngle() {
        double angle = (1.0 - analogEncoder.getAbsolutePosition()) * 2 * Math.PI;
        return ((angle + 3.1415926535) % (2 * 3.1415926535)) - 3.1415926535;
    }
    //in radians
    public double getRawAnalogAngle() {
        double angle = (1.0 - analogEncoder.getAbsolutePosition()) * 2 * Math.PI;
        return angle;
    }

    /*in inches (dont ask why, i dont remember.
    this is the only thing we use inches for)*/
    public double getDistance() {
        return driveEncoder.getPosition();
    }

    public double getRotDistance() {
        return turnEncoder.getPosition();
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(-getDistance(), new Rotation2d(getRotDistance()));
    }
    
}
