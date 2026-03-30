package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import java.util.function.DoubleSupplier;

public class SetShooterYaw extends Command {

  private final DoubleSupplier angleSupplier;
  private final ShooterSubsystem shooter;

  public SetShooterYaw(ShooterSubsystem shooter, DoubleSupplier angleSupplier) {
    this.angleSupplier = angleSupplier;
    this.shooter = shooter;

    addRequirements(shooter);
  }

  @Override
  public void execute() {
    shooter.setYaw(new Rotation2d(angleSupplier.getAsDouble()));
  }
}
