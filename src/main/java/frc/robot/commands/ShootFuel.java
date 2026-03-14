package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.kicker.KickerSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class ShootFuel extends Command {
  private final ShooterSubsystem shooter;
  private final KickerSubsystem kicker;
  private final boolean startShooting;
  private final IntakeSubsystem intake;

  public ShootFuel(
      ShooterSubsystem shooter,
      KickerSubsystem kicker,
      IntakeSubsystem intake,
      boolean startShooting) {
    this.shooter = shooter;
    this.kicker = kicker;
    this.startShooting = startShooting;
    this.intake = intake;
    this.addRequirements(kicker);
  }

  public ShootFuel(ShooterSubsystem shooter, KickerSubsystem kicker, IntakeSubsystem intake) {
    this.shooter = shooter;
    this.kicker = kicker;
    this.startShooting = true;
    this.intake = intake;
    this.addRequirements(kicker);
  }

  @Override
  public void initialize() {
    System.out.println("Starting shooter");
    this.shooter.revShooter();
    this.kicker.startKicker();
    this.intake.startIntake();
    this.shooter.setReadyToShoot(startShooting);
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean wasInterrupted) {
    this.shooter.setReadyToShoot(false);
    this.shooter.stopShooter();
    this.intake.stopIntake();

    this.kicker.stopKicker();
  }
}
