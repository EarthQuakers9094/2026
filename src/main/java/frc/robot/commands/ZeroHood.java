package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class ZeroHood extends Command {

    private final ShooterSubsystem shooter;

    public ZeroHood(ShooterSubsystem shooter) {
        this.shooter = shooter;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        shooter.runHoodDown();
    }

    @Override
    public void end(boolean interrupted) {
        if (!interrupted) {
            shooter.zeroHood();
        }
        shooter.stopHood();
    }

    @Override
    public boolean isFinished() {
        return shooter.getHoodCurrent() >= 30.0;
    }
}
