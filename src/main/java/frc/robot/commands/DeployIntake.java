package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.IntakeSubsystem;

public class DeployIntake extends Command {

  private final IntakeSubsystem intake;

  public DeployIntake(IntakeSubsystem intake) {
    this.intake = intake;

    addRequirements(intake);
  }

  @Override
  public void initialize() {
    intake.deployIntake();
    ;
  }

  @Override
  public void end(boolean _interrupted) {
    intake.retractIntake();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
