package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Rotation;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;

public class IntakeIOReal implements IntakeIO {

  DCMotor spinGearbox = DCMotor.getKrakenX60(1); // make sure this is the right kraken
  DCMotor pivotGearbox = DCMotor.getKrakenX60(1);

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

  public void updateInputs(IntakeIOInputs inputs) {}

  public void runIntake(AngularVelocity rotations) { // rotations per minute
    spinMotor.setControl(new VelocityVoltage(rotations.in(RPM))); // set to something else if needed
  }

  public void pivotIntake(Angle rotation) {
    final PositionVoltage encoder = new PositionVoltage(0).withSlot(0);

    pivotMotor.setControl(encoder.withPosition(rotation.in(Rotation)));
  }
}
