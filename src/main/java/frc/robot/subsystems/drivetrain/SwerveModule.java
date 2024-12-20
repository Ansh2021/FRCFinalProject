package frc.robot.subsystems.drivetrain;

import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;
import frc.robot.MyLibrary.A_Cancoder;
import frc.robot.MyLibrary.A_Motor;

public class SwerveModule {
    private final String MODULE_NAME;
    private final Translation2d LOCATION;
    private final double GEAR_RATIO;
    private final int SPEED_ID;
    private final int ANGLE_ID;
    private final int ENCODER_ID;
    private final double ZERO;
    
    private final A_Motor speed;
    private final A_Motor angle;
    private final A_Cancoder encoder;

    private final PIDController speedPID;
    private final SimpleMotorFeedforward speedFF;
    private final PIDController anglePID;
    private final SimpleMotorFeedforward angleFF;

    public SwerveModule(String moduleName, Translation2d location, double gearRatio, int speedID, int angleID, int encoderID, double zero) {
        this.MODULE_NAME = moduleName;
        this.LOCATION = location;
        this.GEAR_RATIO = gearRatio;
        this.SPEED_ID = speedID;
        this.ANGLE_ID = angleID;
        this.ENCODER_ID = encoderID;
        this.ZERO = zero;

        this.speedPID = Constants.drivetrain.SPEED_PID;
        this.speedFF = new SimpleMotorFeedforward(Constants.drivetrain.SPEED_FF.ks, Constants.drivetrain.SPEED_FF.kv);
        this.anglePID = Constants.drivetrain.ANGLE_PID;
        this.angleFF = Constants.drivetrain.ANGLE_FF;

        this.speed = new A_Motor(SPEED_ID, "canivore");
        this.angle = new A_Motor(ANGLE_ID, "canivore");
        this.encoder = new A_Cancoder(ENCODER_ID, "canivore");

        init();
    }

    public void init() {
        speed.setBrake(false);
        speed.setRampRate(Constants.drivetrain.RAMP_RATE);
        angle.setBrake(false);
        angle.setInverted(true);
        
        // clears any persistent or "sticky" flags/faults on the encoder
        encoder.clearStickyFaults();

        // TODO ask FRC programmers if I need magnet sensor for final project
        MagnetSensorConfigs sensorConfigs = new MagnetSensorConfigs();
        sensorConfigs.MagnetOffset = -(ZERO / 360.0);
        sensorConfigs.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
        sensorConfigs.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        encoder.getConfigurator().apply(sensorConfigs);

        // makes the PID for the angle motors continuous
        // if there is a sudden jump from -180 to 180 or vice versa, the PID would normally break
        // since we are telling the PID to "enableContinuousInput", it treats the jump as a continuous flow so it doesn't break
        anglePID.enableContinuousInput(-180, 180);
        // tolerance just tells the PIDController how much error there still is before it is on the target position
        // I'm guessing that since the tolerance is 0, the PID should be "perfect".
        anglePID.setTolerance(0);
        // just tells the encoder to measure in degrees and gives the encoder the degree value
        angle.setEncoderDegrees(getAngleDegrees());

        // updates velocity, rotor position, and device temp
        // cannot be much faster or else CANbus would die
        speed.getVelocity().setUpdateFrequency(50.0);
        speed.getRotorPosition().setUpdateFrequency(50.0);
        speed.getDeviceTemp().setUpdateFrequency(4.0);
        angle.getVelocity().setUpdateFrequency(50.0);
        angle.getRotorPosition().setUpdateFrequency(50.0);
        angle.getDeviceTemp().setUpdateFrequency(4.0);
        
        // updates absolute position that way robot knows where it is on the field compared to a fixed point
        // needs a fixed reference point like Newton's First Law of Motion
        encoder.getAbsolutePosition().setUpdateFrequency(100.0);

        // optimizes the utilization of CANbus that way it doesn't die
        speed.optimizeBusUtilization();
        angle.optimizeBusUtilization();
        encoder.optimizeBusUtilization();

        // clears any persistent/"sticky" flags/faults
        speed.clearStickyFaults();
        angle.clearStickyFaults();
    }

    // calculates speed needed for each swerve module to reach a specified velocity
    public void setSwerveModuleState(SwerveModuleState optimized) {
        SwerveModuleState optimal = SwerveModuleState.optimize(optimized, Rotation2d.fromDegrees(getAngleDegrees()));

        double velocity = optimal.speedMetersPerSecond / Constants.drivetrain.DRIVE_METERS_PER_TICK;

        // this drivetrain PID allows for smooth driving and movement of the robot
        speed.setVoltage(speedPID.calculate(getVelocity(), velocity) + speedFF.calculate(velocity));
        angle.setVoltage(anglePID.calculate(getAngleDegrees(), optimal.angle.getDegrees()));
    }

    // stops motors
    public void zeroWillToRun() {
        speed.setVoltage(0.0);
        angle.setVoltage(0.0);
    }

    // returns a new Swerve Module object that has the velocity and rotation/angle of the module
    public SwerveModuleState getSwerveModuleState() {
        return new SwerveModuleState(getVelocity(), Rotation2d.fromDegrees(getAngleDegrees()));
    }

    // returns a new Swerve Module object that has the distance that the wheel has traveled and the rotation of the wheel/module
    public SwerveModulePosition getSwerveModulePos() {
        return new SwerveModulePosition(getPosMeters(), Rotation2d.fromDegrees(getAngleDegrees()));
    }

    /**
     * divided by gear ratio because gear ratio is input to output speed of the gears
     * @return robot velocity
     */
    public double getVelocity() {
        return ((speed.getRPM() / 60.0) * Constants.drivetrain.WHEEL_CIRCUMFERENCE) / GEAR_RATIO;
    }

    /**
     * 
     * @return rotation of robot in Rotation2d
     */
    public Rotation2d getAngleRot() {
        return Rotation2d.fromDegrees(getAngleDegrees());
    }
    
    /**
     * 
     * @return rotation of robot in degrees
     */
    public double getAngleDegrees() {
        return encoder.givePosDeg();
    }

    /**
     * 
     * @return setpoint of PID for speed motors
     */
    public double getSpeedPIDSetpoint() {
        return speedPID.getSetpoint();
    }

    /**
     * 
     * @return setpoint of PID for angle motors
     */
    public double getAnglePIDSetpoint() {
        return anglePID.getSetpoint();
    }

    /**
     * 
     * @return position traveled in meters
     */
    public double getPosMeters() {
        return (speed.getPosDeg() * Constants.drivetrain.WHEEL_CIRCUMFERENCE) / (360 * GEAR_RATIO);
    }

    /**
     * 
     * @return location of the swerve module(s)
     */
    public Translation2d getLocation() {
        return this.LOCATION;
    }

    // sets constants for speed motors PID
    public void setSpeedPIDConstants(double kP, double kI, double kD) {
        speedPID.setPID(kP, kI, kD);
    }

    // sets constants for angle motors PID
    public void setAnglePIDConstants(double kP, double kI, double kD) {
        anglePID.setPID(kP, kI, kD);
    }
}
