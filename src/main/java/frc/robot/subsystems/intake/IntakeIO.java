package frc.robot.subsystems.intake;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;

public interface IntakeIO { 
  @AutoLog
  public static class IntakeIOInputs {
    public boolean isIntaking = false;
  }
//if it's in the sim, it needs to be here too (or in the real)
  public default void updateInputs(IntakeIOInputs inputs) {}


  public default void runIntake(double speed){}

  public default void pivotIntake(double angle){}

}
