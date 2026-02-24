package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Rotations;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig;
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
    // pivotMotor
    //     .getConfigurator()
    //     .apply(
    //         new Slot0Configs()
    //             .withKP(Constants.IntakeConstants.pivotkP)
    //             .withKI(0.0)
    //             .withKD(Constants.IntakeConstants.pivotkD)
    //             .withKV(Constants.IntakeConstants.pivotkV));
    pivotMotor.configure(
        new SparkFlexConfig()
            .apply(
                new ClosedLoopConfig()
                    .pid(
                        Constants.IntakeConstants.pivotkP,
                        Constants.IntakeConstants.pivotkI,
                        Constants.IntakeConstants.pivotkD)
                    .apply(new FeedForwardConfig().kV(Constants.IntakeConstants.pivotkV)))
        /*  .apply(
        new EncoderConfig()
            .positionConversionFactor(Constants.IntakeConstants.intakeConversionFactor)),*/ ,
        ResetMode.kResetSafeParameters,
        PersistMode.kNoPersistParameters);

    // find gear ratios
    // spinMotor
    //     .getConfigurator()
    //     .apply(
    //         new Slot0Configs()
    //             .withKP(Constants.IntakeConstants.spinkP)
    //             .withKI(0.0)
    //             .withKD(Constants.IntakeConstants.spinkD)
    //             .withKV(Constants.IntakeConstants.spinkV));
  }

  public void updateInputs(IntakeIOInputs inputs) {

    // boolean isIntaking;
    // if (spinMotor.getSupplyVoltage().getValueAsDouble() == 0.0) {
    //   isIntaking = false;
    // } else {
    //   isIntaking = true;
    // }

    // inputs.isIntaking = isIntaking;
    inputs.pivotAngle = Rotations.of(pivotMotor.getEncoder().getPosition());
  }

  public void runIntake(AngularVelocity rotations) {
    // spinMotor.setControl(new VelocityVoltage(rotations));
  }

  public void pivotIntake(Angle rotation) {
    // pivotMotor.setControl(new PositionVoltage(rotation).withSlot(0));
  }
}
