package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  public static class IntakeIOInputs {
    public boolean isIntaking = false;
  }
  // if it's in the sim, it needs to be here too (or in the real)
  public default void updateInputs(IntakeIOInputs inputs) {}

  public default void runIntake(double speed) {}

  public default void pivotIntake(double angle) {}
}
