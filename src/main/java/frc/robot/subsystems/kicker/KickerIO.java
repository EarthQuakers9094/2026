package frc.robot.subsystems.kicker;

import org.littletonrobotics.junction.AutoLog;

public interface KickerIO {
  @AutoLog
  public static class KickerIOInputs {
    // TODO add auto logged variables which are needed (currently unknown to me)
    public double velocity_sepoint_in_meters = -1d;
    public double current_velocity_in_meters = -1d;
  }

  public default void startKicker() {}

  public default void stopKicker() {}

  public default double getVelocityMeters() {
    return -1.0d;
  }

  public default double getVelocitySetpointMeters() {
    return -1.0d;
  }

  public default void updateInputs(KickerIOInputs inputs) {}
}
