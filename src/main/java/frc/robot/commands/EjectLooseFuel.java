package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class EjectLooseFuel extends SequentialCommandGroup {
  // private final ShooterSubsystem shooter;
  // private final KickerSubsystem kicker;

  public EjectLooseFuel(ShooterSubsystem shooter) {
    super(
        new InstantCommand(shooter::revShooter),
        new WaitCommand(0.5),
        new InstantCommand(shooter::stopShooter));
    // addRequirements(shooter);
  }
}
