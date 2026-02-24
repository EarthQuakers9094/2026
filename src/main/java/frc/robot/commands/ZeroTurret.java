package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class ZeroTurret extends SequentialCommandGroup {

  public ZeroTurret(ShooterSubsystem shooter) {
    super(
        new InstantCommand(shooter::slowlyMoveTowardsLimit),
        new WaitUntilCommand(shooter::isAtTurretLimit),
        new InstantCommand(shooter::zeroTurret));
    addRequirements(shooter);
  }
}
