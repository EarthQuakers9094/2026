package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Rotations;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.FeedForwardConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;

public class IntakeIOReal implements IntakeIO {

  private final SparkFlex pivotMotor =
      new SparkFlex(IntakeConstants.intakePivotMotor, MotorType.kBrushless);
  private final SparkFlex spinMotor =
      new SparkFlex(IntakeConstants.intakeSpinMotor, MotorType.kBrushless);
  // private final TalonFX pivotMotor = new TalonFX(IntakeConstants.intakePivotMotor);
  // private final TalonFX spinMotor = new TalonFX(IntakeConstants.intakeSpinMotor);

  public IntakeIOReal() {
    pivotMotor.configure(
        new SparkFlexConfig()
            .smartCurrentLimit(60)
            .apply(
                new ClosedLoopConfig()
                    .pid(
                        Constants.IntakeConstants.pivotkP,
                        Constants.IntakeConstants.pivotkI,
                        Constants.IntakeConstants.pivotkD)
                    .apply(
                        new FeedForwardConfig()
                            .kV(Constants.IntakeConstants.pivotkV)
                            .kCos(Constants.IntakeConstants.pivotkCos)))
            .apply(
                new EncoderConfig()
                    .positionConversionFactor(
                        Constants.IntakeConstants.intakePivotConversionFactor)),
        ResetMode.kResetSafeParameters,
        PersistMode.kNoPersistParameters);

    spinMotor.configure(
        new SparkFlexConfig()
            .inverted(true)
            .smartCurrentLimit(60)
            .apply(
                new ClosedLoopConfig()
                    .pid(
                        Constants.IntakeConstants.spinkP,
                        Constants.IntakeConstants.spinkI,
                        Constants.IntakeConstants.spinkD)
                    .apply(new FeedForwardConfig().kV(Constants.IntakeConstants.spinkV)))
            .apply(
                new EncoderConfig()
                    .velocityConversionFactor(
                        Constants.IntakeConstants.intakeSpinConversionFactor)),
        ResetMode.kResetSafeParameters,
        PersistMode.kNoPersistParameters);

    pivotMotor.getEncoder().setPosition(Constants.IntakeConstants.startAngle.in(Rotations));
  }

  public void updateInputs(IntakeIOInputs inputs) {
    inputs.intakePivotAngle = Rotations.of(pivotMotor.getEncoder().getPosition());
    inputs.intakePivotCurrent = Amps.of(pivotMotor.getOutputCurrent());
    inputs.intakePivotAngularVelocity = RPM.of(pivotMotor.getEncoder().getVelocity());

    inputs.intakeSpinnerAngularVelocity = RPM.of(spinMotor.getEncoder().getVelocity());
    inputs.intakeSpinnerCurrent = Amps.of(spinMotor.getOutputCurrent());
  }

  public void runIntake(AngularVelocity rotations) {
    // spinMotor.setControl(new VelocityVoltage(rotations));
    spinMotor.getClosedLoopController().setSetpoint(rotations.in(RPM), ControlType.kVelocity);
  }

  public void pivotIntake(Angle rotation) {
    pivotMotor.getClosedLoopController().setSetpoint(rotation.in(Rotations), ControlType.kPosition);
    // pivotMotor.setControl(new PositionVoltage(rotation).withSlot(0));
  }
}
