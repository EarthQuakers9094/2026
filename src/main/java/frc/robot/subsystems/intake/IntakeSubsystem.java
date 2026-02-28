package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

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
  private AngularVelocity speedSetpoint = RPM.of(0);

  public IntakeSubsystem(IntakeIO io) {
    this.io = io;
  }

  public void startIntake() {
    io.runIntake(Constants.IntakeConstants.intakeSpeed);
    this.speedSetpoint = Constants.IntakeConstants.intakeSpeed;
  }

  public void stopIntake() {
    io.runIntake(RPM.of(0));
    this.speedSetpoint = RPM.of(0);
  }

  @Deprecated
  private void pivotIntake(Angle angle) {
    io.pivotIntake(angle);
    this.state = IntakeState.Moving;
    pivotSetpoint = angle;
    Logger.recordOutput("Intake/PivotSetpointRad", angle.in(Radians));
  }

  public void deployIntake() {
    io.pivotIntake(Constants.IntakeConstants.deployedAngle);
    pivotSetpoint = Constants.IntakeConstants.deployedAngle;
    this.targetState = IntakeState.Deployed;
  }

  public void retractIntake() {
    io.pivotIntake(Constants.IntakeConstants.retractedAngle);
    pivotSetpoint = Constants.IntakeConstants.retractedAngle;
    this.targetState = IntakeState.Retracted;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);

    Logger.recordOutput("Intake/State", state);
    Logger.recordOutput("Intake/SpinSpeed", speedSetpoint);

    inputs.intakeSpinnerAngularVelocitySetpoint = speedSetpoint;
    inputs.intakePivotAngleSetpoint = pivotSetpoint;

    if (Math.abs(pivotSetpoint.in(Degrees) - inputs.intakePivotAngle.in(Degrees)) < 4.) {
      this.state = this.targetState;
    } else {
      this.state = IntakeState.Moving;
    }

    Logger.processInputs(getName(), inputs);
  }
}
