package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.kicker.KickerSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class RevvShooter extends Command {
  private final ShooterSubsystem shooter;
  private final KickerSubsystem kicker;

  public RevvShooter(ShooterSubsystem shooter, KickerSubsystem kicker) {
    this.shooter = shooter;
    this.kicker = kicker;
    addRequirements(shooter, kicker);
  }

  @Override
  public void initialize() {
    shooter.revShooter();
    kicker.startKicker();
  }

  @Override
  public void end(boolean wasInterrupted) {}
}
