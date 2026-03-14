package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.IntakeSubsystem;
import java.util.function.BooleanSupplier;

public class RunIntakeSpinnerCommand extends Command {

  private final IntakeSubsystem intake;
  private final BooleanSupplier runForward;

  /**
   * @param intake an IntakeSubsystem
   * @param runForward supply true if intake should run forward, false if backwards
   */
  public RunIntakeSpinnerCommand(IntakeSubsystem intake, BooleanSupplier runForward) {
    this.intake = intake;
    this.runForward = runForward;
    // addRequirements(intake);
  }

  @Override
  public void execute() {
    if (runForward.getAsBoolean()) {
      intake.startIntake();
    } else {
      intake.reverseIntake();
    }
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
