package frc.robot.subsystems.kicker;

import org.littletonrobotics.junction.AutoLog;

public interface KickerIO {
  @AutoLog
  public static class KickerIOInputs {
    // TODO add auto logged variables which are needed (currently unknown to me)
    public double rpm_present = -1d;
    public double rpm_setpoint = -1d;
  }

  public default void startKicker() {}

  public default void stopKicker() {}

  public default double getRPM() {
    return -1.0d;
  }

  public default double getRPMSetpoint() {
    return -1.0d;
  }

  public default void updateInputs(KickerIOInputs inputs) {}
}
