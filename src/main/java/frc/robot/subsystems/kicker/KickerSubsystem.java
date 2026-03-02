package frc.robot.subsystems.kicker;

import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class KickerSubsystem extends SubsystemBase {
  private final KickerIO io;
  private final KickerIOInputsAutoLogged inputs = new KickerIOInputsAutoLogged();

  public KickerSubsystem(KickerIO io) {
    this.io = io;
  }

  public void startKicker() {
    io.startKicker();
  }

  public void stopKicker() {
    io.stopKicker();
  }

  public boolean kickerAtSpeed() {
    double rpm = inputs.angularVelocity.in(RPM);
    double setpoint = inputs.angularVelocitySetpoint.in(RPM);
    double tolerance = Math.abs(setpoint) * 0.05;
    if (setpoint != 0.0d && (rpm >= (setpoint - tolerance)) && (rpm <= (setpoint + tolerance))) {
      return true;
    }
    return false;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(getName(), inputs);
  }
}
