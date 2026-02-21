package frc.robot.subsystems.spindexer;

import edu.wpi.first.units.measure.AngularVelocity;
import org.littletonrobotics.junction.AutoLog;

public interface SpindexerIO {
  @AutoLog
  public static class SpindexerIOInputs {
    public double spindexerCurrentSpeed = 0;
  }

  public default void updateInputs(SpindexerIOInputs inputs) {}

  public default void run(AngularVelocity spindexerSetSpeed) {}
}
