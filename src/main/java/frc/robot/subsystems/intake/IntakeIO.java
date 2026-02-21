package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

public interface IntakeIO {
  @AutoLog
  public static class IntakeIOInputs {
    public boolean isIntaking = false;
    public double intakeVoltage = 0;
  }
  // if it's in the sim, it needs to be here too (or in the real)
  public default void updateInputs(IntakeIOInputs inputs) {}

  public default void runIntake(AngularVelocity speed) {}

  public default void pivotIntake(Angle angle) {}
}
