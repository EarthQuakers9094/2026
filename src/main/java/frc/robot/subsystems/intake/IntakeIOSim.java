package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.FeedForwardConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

public class IntakeIOSim implements IntakeIO {

  private final DCMotor spinDCMotor;
  private final DCMotor pivotDCMotor;

  private final SparkFlex spinMotor;
  private final SparkFlex pivotMotor;

  private final SparkFlexSim spinMotorSim;
  private final SparkFlexSim pivotMotorSim;

  private SingleJointedArmSim armSim;
  private FlywheelSim spinSim;

  private final LoggedMechanism2d armMechanism = new LoggedMechanism2d(3, 3);
  private final LoggedMechanismRoot2d armMechanismRoot =
      armMechanism.getRoot("intakeArm", 1.5, 1.5);
  private final LoggedMechanismLigament2d intakeArmVisualization =
      armMechanismRoot.append(
          new LoggedMechanismLigament2d(
              "intakeArm", Constants.IntakeConstants.armLength.in(Meters), 0));

  public IntakeIOSim() {
    spinDCMotor = DCMotor.getNeoVortex(1);
    pivotDCMotor = DCMotor.getNeoVortex(1);

    spinMotor = new SparkFlex(Constants.IntakeConstants.intakeSpinMotor, MotorType.kBrushless);
    spinMotor.configure(
        new SparkFlexConfig()
            .smartCurrentLimit(60)
            .apply(
                new EncoderConfig()
                    .velocityConversionFactor(Constants.IntakeConstants.intakeSpinConversionFactor))
            .apply(
                new ClosedLoopConfig()
                    .pid(
                        Constants.IntakeConstants.spinkP,
                        Constants.IntakeConstants.spinkI,
                        Constants.IntakeConstants.spinkD)
                    .apply(new FeedForwardConfig().kV(Constants.IntakeConstants.spinkV))),
        ResetMode.kResetSafeParameters,
        PersistMode.kNoPersistParameters);

    pivotMotor = new SparkFlex(Constants.IntakeConstants.intakePivotMotor, MotorType.kBrushless);
    pivotMotor.configure(
        new SparkFlexConfig()
            .smartCurrentLimit(60)
            .inverted(true)
            .apply(
                new EncoderConfig()
                    .positionConversionFactor(
                        Constants.IntakeConstants.intakePivotConversionFactor))
            .apply(
                new ClosedLoopConfig()
                    .pid(
                        Constants.IntakeConstants.pivotkP,
                        Constants.IntakeConstants.pivotkI,
                        Constants.IntakeConstants.pivotkD)
                    .apply(
                        new FeedForwardConfig()
                            .kCos(Constants.IntakeConstants.pivotkCos)
                            .kV(Constants.IntakeConstants.pivotkV))),
        ResetMode.kResetSafeParameters,
        PersistMode.kNoPersistParameters);

    spinMotorSim = new SparkFlexSim(spinMotor, spinDCMotor);
    pivotMotorSim = new SparkFlexSim(pivotMotor, pivotDCMotor);

    armSim =
        new SingleJointedArmSim(
            LinearSystemId.createSingleJointedArmSystem(
                pivotDCMotor,
                Constants.IntakeConstants.pivotMOI,
                Constants.IntakeConstants.intakePivotGearing),
            pivotDCMotor,
            Constants.IntakeConstants.intakePivotGearing,
            Constants.IntakeConstants.armLength.in(Meters),
            Constants.IntakeConstants.minAngle.in(Radians),
            Constants.IntakeConstants.maxAngle.in(Radians),
            true,
            Constants.IntakeConstants.startAngle.in(Radians));

    spinSim =
        new FlywheelSim(
            LinearSystemId.createFlywheelSystem(
                spinDCMotor,
                Constants.IntakeConstants.spinMOI,
                Constants.IntakeConstants.intakeSpinGearing),
            spinDCMotor);
  }

  public void updateInputs(IntakeIOInputs inputs) {
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(
            armSim.getCurrentDrawAmps(), spinSim.getCurrentDrawAmps()));

    double pivotVoltage = pivotMotorSim.getAppliedOutput() * RoboRioSim.getVInVoltage();
    armSim.setInputVoltage(pivotVoltage);
    armSim.update(0.02);

    double spinVoltage = spinMotorSim.getAppliedOutput() * RoboRioSim.getVInVoltage();
    spinSim.setInputVoltage(spinVoltage);
    spinSim.update(0.02);

    Angle armAngle = Radians.of(armSim.getAngleRads());
    inputs.intakePivotAngle = armAngle;
    intakeArmVisualization.setAngle(armAngle);
    pivotMotorSim.setPosition(armAngle.in(Rotations));

    AngularVelocity pivotAngularVelocity = RadiansPerSecond.of(armSim.getVelocityRadPerSec());
    inputs.intakePivotAngularVelocity = pivotAngularVelocity;
    pivotMotorSim.setVelocity(pivotAngularVelocity.in(RPM));

    AngularVelocity spinAngularVelocity = spinSim.getAngularVelocity();
    spinMotorSim.setVelocity(spinAngularVelocity.in(RPM));
    inputs.intakeSpinnerAngularVelocity = spinAngularVelocity;

    inputs.intakePivotCurrent = Amps.of(pivotMotor.getAppliedOutput());
    inputs.intakeSpinnerCurrent = Amps.of(spinMotor.getAppliedOutput());

    pivotMotorSim.iterate(
        RadiansPerSecond.of(armSim.getVelocityRadPerSec()).in(RPM),
        RoboRioSim.getVInVoltage(),
        0.02);
    spinMotorSim.iterate(spinSim.getAngularVelocityRPM(), RoboRioSim.getVInVoltage(), 0.02);

    Logger.recordOutput("Intake/Arm", armMechanism);
  }

  public void runIntake(AngularVelocity rotations) {
    spinMotor.getClosedLoopController().setSetpoint(rotations.in(RPM), ControlType.kVelocity);
  }

  public void pivotIntake(Angle rotation) {
    pivotMotor.getClosedLoopController().setSetpoint(rotation.in(Rotations), ControlType.kPosition);
  }
}
