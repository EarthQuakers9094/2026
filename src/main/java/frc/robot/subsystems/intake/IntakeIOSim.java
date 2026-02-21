package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotation;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.sim.TalonFXSimState.MotorType;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;

public class IntakeIOSim implements IntakeIO {

  DCMotor spinGearbox = DCMotor.getKrakenX60(1); // make sure this is the right kraken
  DCMotor pivotGearbox = DCMotor.getKrakenX60(1);

  private final TalonFX pivotMotor = new TalonFX(IntakeConstants.intakePivotMotor);
  private final TalonFX spinMotor = new TalonFX(IntakeConstants.intakeSpinMotor);

  private final TalonFXSimState pivotMotorSim;
  private final TalonFXSimState intakeMotorSim;

  private SingleJointedArmSim armSim =
      new SingleJointedArmSim(
          LinearSystemId.createSingleJointedArmSystem(
              pivotGearbox,
              Constants.IntakeConstants.intakeMOI,
              Constants.IntakeConstants.intakeGearing),
          pivotGearbox,
          Constants.IntakeConstants.intakeGearing,
          Constants.IntakeConstants.armLength.in(Meters),
          0,
          Constants.IntakeConstants.maxAngle.in(Radians),
          true,
          0);

  public IntakeIOSim() {
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

    this.intakeMotorSim = spinMotor.getSimState();
    this.pivotMotorSim = pivotMotor.getSimState();
    intakeMotorSim.setMotorType(MotorType.KrakenX60);
    pivotMotorSim.setMotorType(MotorType.KrakenX60);
  }

  public void updateInputs(IntakeIOInputs inputs) {

    intakeMotorSim.setSupplyVoltage(RoboRioSim.getVInVoltage());
    pivotMotorSim.setSupplyVoltage(RoboRioSim.getVInVoltage());

    armSim.update(0.02);

    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(armSim.getCurrentDrawAmps()));

    boolean isIntaking;
    if (intakeMotorSim.getMotorVoltage() == 0) {
      isIntaking = false;
    } else {
      isIntaking = true;
    }

    inputs.isIntaking = isIntaking;
  }

  public void runIntake(AngularVelocity rotations) { // rotations per minute
    spinMotor.setControl(new VelocityVoltage(rotations)); 
  }

  public void pivotIntake(Angle rotation) {
    pivotMotor.setControl(new PositionVoltage(rotation).withSlot(0));
  }
}
