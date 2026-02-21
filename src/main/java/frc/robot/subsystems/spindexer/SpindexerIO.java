package frc.robot.subsystems.spindexer;

import org.littletonrobotics.junction.AutoLog;

public interface SpindexerIO {
  @AutoLog
  public static class SpindexerIOInputs {
    public double spindexerSetSpeed = 0;
  }

  public default void updateInputs(SpindexerIOInputs inputs) {}

  public default void setVelocitySetpoint(double spindexerSpeed) {}
}
