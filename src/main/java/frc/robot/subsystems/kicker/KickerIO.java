package frc.robot.subsystems.kicker;

import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.units.measure.AngularVelocity;
import org.littletonrobotics.junction.AutoLog;

public interface KickerIO {
  @AutoLog
  public static class KickerIOInputs {
    public AngularVelocity angularVelocity = RPM.of(0d);
    public AngularVelocity angularVelocitySetpoint = RPM.of(0d);
  }

  public default void startKicker() {}

  public default void stopKicker() {}

  public default void updateInputs(KickerIOInputs inputs) {}
}
