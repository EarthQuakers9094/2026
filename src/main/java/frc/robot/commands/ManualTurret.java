package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class ManualTurret extends Command {

    private final DoubleSupplier rotationSupplier;
    private final ShooterSubsystem shooter;
    
    public ManualTurret(ShooterSubsystem shooter, DoubleSupplier rotationSupplier) {
        this.rotationSupplier = rotationSupplier;
        this.shooter = shooter;
        addRequirements(shooter);
    }

    @Override
    public void execute() {
        shooter.setYaw(new Rotation2d(shooter.getYaw().plus(Degrees.of(rotationSupplier.getAsDouble()))));
    }
}
