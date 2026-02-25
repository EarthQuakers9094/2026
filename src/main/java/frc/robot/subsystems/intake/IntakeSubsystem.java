package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

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
  private AngularVelocity intakeSetpoint = RPM.of(0);

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
    this.targetState = IntakeState.Deployed;
    this.pivotIntake(Constants.IntakeConstants.deployedAngle);
  }

  public void retractIntake() {
    this.targetState = IntakeState.Retracted;
    this.pivotIntake(Constants.IntakeConstants.retractedAngle);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);

    Logger.recordOutput("Intake/State", state);
    Logger.recordOutput("Intake/SpinSpeedRadPerSec", intakeSetpoint.in(RadiansPerSecond));

    if (Math.abs(pivotSetpoint.in(Degrees) - inputs.pivotAngle.in(Degrees)) < 4.) {
      this.state = this.targetState;
    } else {
      this.state = IntakeState.Moving;
    }

    Logger.processInputs("Intake", inputs);
  }
}
