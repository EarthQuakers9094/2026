package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  public static class IntakeIOInputs {
    // Intake Spinner
    public AngularVelocity intakeSpinnerAngularVelocity = RPM.of(0);
    public AngularVelocity intakeSpinnerAngularVelocitySetpoint = RPM.of(0);
    public Current intakeSpinnerCurrent = Amps.of(0);

    // Intake Pivot
    public AngularVelocity intakePivotAngularVelocity = RPM.of(0);
    public Angle intakePivotAngle = Rotations.of(0);
    public Angle intakePivotAngleSetpoint = Rotations.of(0);
    public Current intakePivotCurrent = Amps.of(0);
  }

  public default void updateInputs(IntakeIOInputs inputs) {}

  public default void runIntake(AngularVelocity speed) {}

  public default void pivotIntake(Angle angle) {}
}
