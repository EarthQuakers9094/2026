package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.intake.IntakeSubsystem;

public class IntakeFuel extends Command {

  private final IntakeSubsystem intake;

  public IntakeFuel(IntakeSubsystem intake) {
    this.intake = intake;

    addRequirements(intake);
  }

  @Override
  public void initialize() {
    intake.pivotIntake(Constants.IntakeConstants.deployedAngle);
    intake.runIntake(Constants.IntakeConstants.intakeSpeed);
  }

  @Override
  public void end(boolean _interrupted) {
    intake.pivotIntake(Constants.IntakeConstants.retractedAngle);
    intake.stopIntake();
  }
}
