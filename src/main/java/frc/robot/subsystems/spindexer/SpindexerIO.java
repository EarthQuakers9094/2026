package frc.robot.subsystems.spindexer;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.AngularVelocity;

public interface SpindexerIO {
  @AutoLog
  public static class SpindexerIOInputs {
    public double spindexerCurrentSpeed = 0;
  }

  public default void updateInputs(SpindexerIOInputs inputs) {}

  public default void run(AngularVelocity spindexerSetSpeed) {}
}
