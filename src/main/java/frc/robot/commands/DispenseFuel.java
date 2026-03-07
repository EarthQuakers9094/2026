package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.IntakeSubsystem;

public class DispenseFuel extends Command {

  private final IntakeSubsystem intake;

  public DispenseFuel(IntakeSubsystem intake) {
    this.intake = intake;

    // addRequirements(intake);
  }

  @Override
  public void initialize() {
    intake.reverseIntake();
  }

  @Override
  public void end(boolean _interrupted) {
    intake.stopIntake();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
