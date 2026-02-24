package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;

public class IntakeIOReal implements IntakeIO {

  private final TalonFX pivotMotor = new TalonFX(IntakeConstants.intakePivotMotor);
  private final TalonFX spinMotor = new TalonFX(IntakeConstants.intakeSpinMotor);

  public IntakeIOReal() {
    pivotMotor
        .getConfigurator()
        .apply(
            new Slot0Configs()
                .withKP(Constants.IntakeConstants.pivotkP)
                .withKI(0.0)
                .withKD(Constants.IntakeConstants.pivotkD)
                .withKV(Constants.IntakeConstants.pivotkV));

    // find gear ratios
    spinMotor
        .getConfigurator()
        .apply(
            new Slot0Configs()
                .withKP(Constants.IntakeConstants.spinkP)
                .withKI(0.0)
                .withKD(Constants.IntakeConstants.spinkD)
                .withKV(Constants.IntakeConstants.spinkV));
  }

  public void updateInputs(IntakeIOInputs inputs) {

    boolean isIntaking;
    if (spinMotor.getSupplyVoltage().getValueAsDouble() == 0.0) {
      isIntaking = false;
    } else {
      isIntaking = true;
    }

    inputs.isIntaking = isIntaking;
  }

  public void runIntake(AngularVelocity rotations) {
    spinMotor.setControl(new VelocityVoltage(rotations));
  }

  public void pivotIntake(Angle rotation) {
    pivotMotor.setControl(new PositionVoltage(rotation).withSlot(0));
  }
}
