package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.kicker.KickerSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class ShootFuel extends Command {
  private final ShooterSubsystem shooter;
  private final KickerSubsystem kicker;
  private final boolean startShooting;

  public ShootFuel(ShooterSubsystem shooter, KickerSubsystem kicker, boolean startShooting) {
    this.shooter = shooter;
    this.kicker = kicker;
    this.startShooting = startShooting;
    this.addRequirements(shooter, kicker);
  }

  public ShootFuel(ShooterSubsystem shooter, KickerSubsystem kicker) {
    this.shooter = shooter;
    this.kicker = kicker;
    this.startShooting = true;
    this.addRequirements(shooter, kicker);
  }

  @Override
  public void initialize() {
    System.out.println("Starting shooter");
    this.shooter.revShooter();
    this.kicker.startKicker();
    this.shooter.setReadyToShoot(startShooting);
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean wasInterrupted) {
    this.shooter.setReadyToShoot(false);
    this.shooter.stopShooter();
    this.kicker.stopKicker();
  }
}
