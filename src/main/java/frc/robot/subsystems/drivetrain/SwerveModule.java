package frc.robot.subsystems.drivetrain;

import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;
import frc.robot.commands.Motorstuff;

public class SwerveModule {
  private final String MODULE_NAME;
  private final Translation2d LOCATION;
  private final double GEAR_RATIO;
  private final int SPEED_ID;
  private final int ANGLE_ID;
  private final int ENCODER_ID;
  private final double ZERO;

  private final Motorstuff speed;
  private final Motorstuff angle;
  private final CANcoder encoder;

  private final PIDController anglePID;
  private final SimpleMotorFeedforward angleFF;
  private final PIDController speedPID;
  private final SimpleMotorFeedforward speedFF;

  public SwerveModule(String moduleName, Translation2d location, double gearRatio, int SpeedID, int AngleID, int EncoderID, double zero) {
    this.MODULE_NAME = moduleName;
    this.LOCATION = location;
    this.GEAR_RATIO = gearRatio;
    this.SPEED_ID = SpeedID;
    this.ANGLE_ID = AngleID;
    this.ENCODER_ID = EncoderID;
    this.ZERO = zero;

    this.anglePID = Constants.drivetrain.ANGLE_PID;
    this.angleFF = Constants.drivetrain.ANGLE_FF;
    this.speedPID = Constants.drivetrain.SPEED_PID;
    this.speedFF = new SimpleMotorFeedforward(Constants.drivetrain.SPEED_FF.ks, Constants.drivetrain.SPEED_FF.kv);

    this.speed = new Motorstuff(SPEED_ID, "canivore");
    this.angle = new Motorstuff(ANGLE_ID, "canivore");
    this.encoder = new CANcoder(ENCODER_ID, "canivore");

    init();
  }

  public void init() {
    speed.setBrake(false);
    speed.setRampRate(Constants.drivetrain.RAMP_RATE);

    angle.setBrake(false);
    angle.setInverted(true);

    encoder.clearStickyFaults();
    // MagnetSensorConfigs sensorConfigs = new MagnetSensorConfigs();
    // sensorConfigs.MagnetOffset = -(ZERO / 360.0);
    // sensorConfigs.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
    // sensorConfigs.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    // encoder.getConfigurator().apply(sensorConfigs);
    
    anglePID.enableContinuousInput(-180, 180);
    anglePID.setTolerance(0);
    angle.setEncoderDegrees(getAngleDegrees());

    speed.getVelocity().setUpdateFrequency(50.0);
    speed.getRotorPosition().setUpdateFrequency(50.0);
    speed.getDeviceTemp().setUpdateFrequency(4.0);

    angle.getVelocity().setUpdateFrequency(50.0);
    angle.getRotorPosition().setUpdateFrequency(50.0);
    angle.getDeviceTemp().setUpdateFrequency(4.0);

    encoder.getAbsolutePosition().setUpdateFrequency(100.0);

    speed.optimizeBusUtilization();
    angle.optimizeBusUtilization();
    encoder.optimizeBusUtilization();

    speed.clearStickyFaults();
    angle.clearStickyFaults();
}

    public void setModuleState(SwerveModuleState needed) {
        SwerveModuleState optimal = SwerveModuleState.optimize(needed, Rotation2d.fromDegrees(getAngleDegrees()));

        double velo = optimal.speedMetersPerSecond / Constants.drivetrain.DRIVE_METERS_PER_TICK;

        speed.setVoltage(speedPID.calculate(getVelocity(), velo) + speedFF.calculate(velo));
        angle.setVoltage(anglePID.calculate(getAngleDegrees(), optimal.angle.getDegrees()));
    }

    public void zeroWillToRun() {
        speed.setVoltage(0.0);
        angle.setVoltage(0.0);
    }
}