package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.IntakeSubsystem;

public class RunIntakeCommand extends Command {

  private final IntakeSubsystem intake;
  private final double speed;

  public RunIntakeCommand(IntakeSubsystem intake, double speed) {
    this.intake = intake;
    this.speed = speed;
  }

  @Override
  public void initialize() {
    intake.runIntake(speed);
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean wasInterrupted) {
    intake.stopIntake();
  }
}
