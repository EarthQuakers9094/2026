package frc.robot.subsystems.kicker;

import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.units.measure.AngularVelocity;
import org.littletonrobotics.junction.AutoLog;

public interface KickerIO {
  @AutoLog
  public static class KickerIOInputs {
    public AngularVelocity angularVelocityCurrent = RPM.of(-1);
    public AngularVelocity angularVelocitySetpoint = RPM.of(-1);
  }

  public default void startKicker() {}

  public default void stopKicker() {}

  public default void updateInputs(KickerIOInputs inputs) {}
}
