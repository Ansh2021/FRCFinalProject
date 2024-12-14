package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Flywheel;

public class FlyFlywheel extends Command {
    Flywheel flywheel = Flywheel.getInstance();

    public FlyFlywheel() {
        addRequirements(flywheel);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        flywheel.run(1.0);
    }

    @Override
    public void end(boolean interrupted) {
        flywheel.run(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}