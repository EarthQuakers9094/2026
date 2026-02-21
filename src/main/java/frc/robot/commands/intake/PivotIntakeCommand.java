package frc.robot.commands.intake;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.IntakeSubsystem;

public class PivotIntakeCommand extends Command {

  private final IntakeSubsystem intake;
  private final Angle rotation;

  public PivotIntakeCommand(IntakeSubsystem intake, Angle rotation) {
    this.intake = intake;
    this.rotation = rotation;
  }

  @Override
  public void initialize() {
    intake.pivotIntake(rotation);
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean wasInterrupted) {}
}
