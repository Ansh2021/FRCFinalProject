package frc.robot.subsystems.drivetrain;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Swerve extends SubsystemBase{
    // creates instance of Swerve if there isn't one already
    private static Swerve instance = null;
    public static synchronized Swerve getInstance() {
        if (instance == null) instance = new Swerve();
        return instance;
    }

    // makes the pigeon object and links it to canbus
    private final Pigeon2 pigeon = new Pigeon2(Constants.ids.PIGEON, "canivore");    
    // makes the poseEstimator
    private final SwerveDrivePoseEstimator poseEstimator = new SwerveDrivePoseEstimator(
        Constants.drivetrain.KINEMATICS, getPigeonRotation(), getSwerveModulePos(), 
        new Pose2d(0, 0, getPigeonRotation()));

    /**
     * why is this the best way to return multiple values in java?
     * @param swervemodules
     * @return locations of swerve modules
     */
    private static Translation2d[] getLocations(SwerveModule... swervemodules) {
        Translation2d[] locations = new Translation2d[swervemodules.length];
        for (int i = 0; i < swervemodules.length; i++) {
            locations[i] = swervemodules[i].getLocation();
        }
        return locations;
    }

    // creates the swerve modules
    private SwerveModule[] moduleList = {
        new SwerveModule (
            "FL",
            Constants.drivetrain.FL_LOCATION,
            Constants.drivetrain.DRIVE_GEARING,
            Constants.ids.FL_SPEED,
            Constants.ids.FL_ANGLE,
            Constants.ids.FL_ENCODER,
            Constants.drivetrain.FL_ZERO
        ) , 
        
        new SwerveModule (
            "FR",
            Constants.drivetrain.FR_LOCATION,
            Constants.drivetrain.DRIVE_GEARING,
            Constants.ids.FR_SPEED,
            Constants.ids.FR_ANGLE,
            Constants.ids.FR_ENCODER,
            Constants.drivetrain.FR_ZERO
        ) ,

        new SwerveModule (
            "BL",
            Constants.drivetrain.BL_LOCATION,
            Constants.drivetrain.DRIVE_GEARING,
            Constants.ids.BL_SPEED,
            Constants.ids.BL_ANGLE,
            Constants.ids.BL_ENCODER,
            Constants.drivetrain.BL_ZERO
        ) ,

        new SwerveModule (
            "BR",
            Constants.drivetrain.BR_LOCATION,
            Constants.drivetrain.DRIVE_GEARING,
            Constants.ids.BR_SPEED,
            Constants.ids.BR_ANGLE,
            Constants.ids.BR_ENCODER,
            Constants.drivetrain.BR_ZERO
        )
    };

    /**
     * 
     * @param swervemodules
     * @return position of swerve module
     */
    private SwerveModulePosition[] getSwerveModulePos(SwerveModule... swervemodules) {
        SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
        for (int i = 0; i < swervemodules.length; i++) {
            modulePositions[i] = swervemodules[i].getSwerveModulePos();
        }
        return modulePositions;
    }

    /**
     * sets module states based off of calculations
     * @param states
     */
    private void setModuleStates(SwerveModuleState[] states) {
        SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.drivetrain.MAX_VELOCITY);
        for (int i = 0; i < 4; i++) {
            states[i] = SwerveModuleState.optimize(states[i], moduleList[i].getAngleRot());
            moduleList[i].setSwerveModuleState(states[i]);
        }
    }
    
    // makes new swerve drive kinematics
    private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(getLocations(moduleList));

    // gets the rotation of the robot
    public Rotation2d getRotation() {
        return poseEstimator.getEstimatedPosition().getRotation();
    }

    /**
     * 
     * @param degrees
     * @return normalized degrees
     */
    public double normalizeDegrees(double degrees) {
        degrees = degrees % 360;
        if (degrees > 180) {
            degrees -= 360;
        } else if (degrees < -180) {
            degrees += 360;
        }
        return degrees;
    }

    /**
     * 
     * @return normalized angle based off of pigeon
     */
    public double getPigeonAngle() {
        return normalizeDegrees(pigeon.getAngle());
    }

    /**
     * 
     * @return normalized angle in Rotation2d based off of pigeon
     */
    public Rotation2d getPigeonRotation() {
        return Rotation2d.fromDegrees(getPigeonAngle());
    }

    /**
     * should set drive speeds
     * @param vx
     * @param vy
     * @param omega
     */
    public void drive(double vx, double vy, double omega) {
        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy, omega, getRotation());
        SwerveModuleState[] states = Constants.drivetrain.KINEMATICS.toSwerveModuleStates(speeds);
        setModuleStates(states);
    }
}
