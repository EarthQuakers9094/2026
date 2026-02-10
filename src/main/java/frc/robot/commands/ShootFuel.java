package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class ShootFuel extends Command {

  private final ShooterSubsystem shooter;

  public ShootFuel(ShooterSubsystem shooter) {
    this.shooter = shooter;
  }

  @Override
  public void initialize() {
    shooter.beginShooting();
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean wasInterrupted) {
    shooter.endShooting();
  }
}
