package frc.robot.commands.intake;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.IntakeSubsystem;

public class RunIntakeCommand extends Command {

  private final IntakeSubsystem intake;
  private final AngularVelocity speed;

  public RunIntakeCommand(IntakeSubsystem intake, AngularVelocity speed) {
    this.intake = intake;
    this.speed = speed;
  }

  @Override
  public void initialize() {
    intake.startIntake();
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean wasInterrupted) {
    intake.stopIntake();
  }
}
