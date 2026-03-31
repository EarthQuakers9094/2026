package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.kicker.KickerSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class RevvShooter extends Command {
  private final ShooterSubsystem shooter;
  private final KickerSubsystem kicker;
  private final IntakeSubsystem intake;

  public RevvShooter(ShooterSubsystem shooter, KickerSubsystem kicker, IntakeSubsystem intake) {
    this.shooter = shooter;
    this.kicker = kicker;
    this.intake = intake;
    addRequirements(kicker);
  }

  @Override
  public void initialize() {
    this.shooter.revShooter();
    this.kicker.startKicker();
    this.intake.startIntake();
    this.shooter.setReadyToShoot(false);
  }

  @Override
  public void end(boolean wasInterrupted) {}
}
