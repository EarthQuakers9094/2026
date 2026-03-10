package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

public class IntakeSubsystem extends SubsystemBase {

  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  public enum IntakeState {
    Deployed,
    Moving,
    Retracted
  }

  public IntakeState state = IntakeState.Retracted;
  public IntakeState targetState = IntakeState.Retracted;
  private Angle pivotSetpoint = Constants.IntakeConstants.startAngle;
  private AngularVelocity intakeSetpoint = RPM.of(0);
  private final LoggedMechanism2d armMechanism = new LoggedMechanism2d(3, 3);
  private final LoggedMechanismRoot2d armMechanismRoot =
      armMechanism.getRoot("intakeArm", 1.0, 0.1);
  private final LoggedMechanismLigament2d intakeArmVisualization =
      armMechanismRoot.append(
          new LoggedMechanismLigament2d(
              "intakeArm", Constants.IntakeConstants.armLength.in(Meters), 0));

  public IntakeSubsystem(IntakeIO io) {
    this.io = io;
  }

  public void runIntake(AngularVelocity speed) {
    this.intakeSetpoint = speed;
    io.runIntake(speed);
  }

  public void startIntake() {
    this.runIntake(Constants.IntakeConstants.intakeSpeed);
  }

  public void reverseIntake() {
    this.runIntake(Constants.IntakeConstants.intakeSpeed.unaryMinus());
  }

  public void stopIntake() {
    io.runIntake(RPM.of(0));
  }

  private void pivotIntake(Angle angle) {
    io.pivotIntake(angle);
    this.state = IntakeState.Moving;
    pivotSetpoint = angle;
    Logger.recordOutput("Intake/PivotSetpointRad", angle.in(Radians));
  }

  public void deployIntake() {
    DriverStation.reportError("DEPLOYED UINTAKE", false);
    this.targetState = IntakeState.Deployed;
    this.pivotIntake(Constants.IntakeConstants.deployedAngle);
  }

  public void retractIntake() {
    this.targetState = IntakeState.Retracted;
    this.pivotIntake(Constants.IntakeConstants.retractedAngle);
  }

  public void setIntakePosition(Angle position) {
    io.setIntakePosition(position);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);

    intakeArmVisualization.setAngle(inputs.pivotAngle.unaryMinus().plus(Degrees.of(180.0)));

    Logger.recordOutput("Intake/State", state);
    Logger.recordOutput("Intake/SpinSpeedRadPerSec", intakeSetpoint.in(RadiansPerSecond));
    Logger.recordOutput("Intake/Visualization", armMechanism);

    if (Math.abs(pivotSetpoint.in(Degrees) - inputs.pivotAngle.in(Degrees)) < 8.) {
      this.state = this.targetState;
    } else {
      this.state = IntakeState.Moving;
    }
  }
}
