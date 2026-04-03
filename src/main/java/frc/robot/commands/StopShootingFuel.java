package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.kicker.KickerSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class StopShootingFuel extends Command {
  private final ShooterSubsystem shooter;
  private final KickerSubsystem kicker;
  private final IntakeSubsystem intake;

  public StopShootingFuel(
      ShooterSubsystem shooter, KickerSubsystem kicker, IntakeSubsystem intake) {
    this.shooter = shooter;
    this.kicker = kicker;
    this.intake = intake;
    this.addRequirements(kicker);
  }

  @Override
  public void initialize() {
    this.shooter.setReadyToShoot(false);
    this.shooter.stopShooter();
    this.intake.stopIntake();

    this.kicker.stopKicker();
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
