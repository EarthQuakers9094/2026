package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class IntakeSubsystem extends SubsystemBase {

  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  public IntakeSubsystem(IntakeIO io) {
    this.io = io;
  }

  public void runIntake(double speed) {
    io.runIntake(speed);
  }

  public void stopIntake() {
    io.runIntake(0);
  }

  public void pivotIntake(double angle) {
    io.pivotIntake(angle);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);

    Logger.processInputs("Intake", inputs);
  }
}
