package frc.robot.subsystems.spindexer;

import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.units.measure.AngularVelocity;
import org.littletonrobotics.junction.AutoLog;

public interface SpindexerIO {
  @AutoLog
  public static class SpindexerIOInputs {
    public AngularVelocity spindexerCurrentSpeed = RPM.of(0);
    public AngularVelocity spinexerVelocitySetpoint = RPM.of(0);
  }

  public default void updateInputs(SpindexerIOInputs inputs) {}

  public default void run(AngularVelocity spindexerSetSpeed) {}
}
