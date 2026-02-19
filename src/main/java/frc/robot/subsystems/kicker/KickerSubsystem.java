package frc.robot.subsystems.kicker;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class KickerSubsystem extends SubsystemBase {
  private final KickerIO io;
  private final KickerIOInputsAutoLogged autoLogged = new KickerIOInputsAutoLogged();

  public KickerSubsystem(KickerIO io) {
    this.io = io;
  }

  public void startKicker() {
    io.startKicker();
  }

  public void stopKicker() {
    io.stopKicker();
  }

  public double getVelocityMeters() {
    return io.getVelocityMeters();
  }

  public double getVelocitySetpointMeters() {
    return io.getVelocitySetpointMeters();
  }

  @Override
  public void periodic() {
    io.updateInputs(autoLogged);
    Logger.processInputs(getName(), autoLogged);
  }
}
