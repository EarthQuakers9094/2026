package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  public static class IntakeIOInputs {
    public boolean isIntaking = false;
    public double intakeVoltage = 0;
    public Angle pivotAngle = Degrees.of(0.0);
  }

  // if it's in the sim, it needs to be here too (or in the real)
  public default void updateInputs(IntakeIOInputs inputs) {}

  public default void runIntake(AngularVelocity speed) {}

  public default void pivotIntake(Angle angle) {}
}
