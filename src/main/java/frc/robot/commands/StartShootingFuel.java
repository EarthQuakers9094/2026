package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.kicker.KickerSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class StartShootingFuel extends Command {
  private final ShooterSubsystem shooter;
  private final KickerSubsystem kicker;
  private final IntakeSubsystem intake;

  public StartShootingFuel(
      ShooterSubsystem shooter, KickerSubsystem kicker, IntakeSubsystem intake) {
    this.shooter = shooter;
    this.kicker = kicker;
    this.intake = intake;
    this.addRequirements(kicker);
  }

  @Override
  public void initialize() {
    System.out.println("Starting shooter");
    this.shooter.revShooter();
    this.kicker.startKicker();
    this.intake.startIntake();
    this.shooter.setReadyToShoot(true);
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
