package frc.robot.subsystems.drivetrain;

import edu.wpi.first.math.geometry.Translation2d;

public class Swerve {
    private static Swerve instance = null;
    public static synchronized Swerve getInstance() {
        if (instance == null) instance = new Swerve(); 
        return instance;
    }

    private static Translation2d[] getLocation(SwerveModule... modules) {
        Translation2d[] locations = new Translation2d[modules.length];
        for (int = 0; i < modules.length; i++) {
            locations[i] = modules[i].getLocation();
        }
        return locations;
    }
}
