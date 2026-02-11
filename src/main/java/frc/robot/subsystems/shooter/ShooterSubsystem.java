package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.MovingAverage;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class ShooterSubsystem extends SubsystemBase {

  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

  private final MovingAverage speedAverage = new MovingAverage(40);
  private double currentAverageSpeed;

  private boolean wasFuelInShooter = false;
  @AutoLogOutput public int shotCount = 0;

  public enum ShooterState {
    Inactive,
    Revving,
    Shooting
  }

  @AutoLogOutput private ShooterState shooterState = ShooterState.Inactive;

  public ShooterSubsystem(ShooterIO io) {
    Logger.recordOutput("ShooterSubsystem/ShootingIsHappening", 0);

    this.io = io;
  }

  private void setSpeedSetpoint(AngularVelocity speed) {
    io.setVelocitySetpoint(speed);
    Logger.recordOutput("Shooter/SpeedSetpointRadPerSec", speed.in(RadiansPerSecond));
  }

  public void beginShooting() {
    if (shooterState == ShooterState.Inactive) {
      Logger.recordOutput("ShooterSubsystem/ShootingIsHappening", 1);

      shooterState = ShooterState.Revving;
      setSpeedSetpoint(Constants.ShooterConstants.launchSpeed);
    }
  }

  public void endShooting() {
    Logger.recordOutput("ShooterSubsystem/ShootingIsHappening", -1);
    shooterState = ShooterState.Inactive;
    setSpeedSetpoint(RPM.of(0.));
  }

  public boolean isActivelyShooting() {
    return shooterState == ShooterState.Shooting;
  }

  @Override
  public void periodic() {

    io.updateInputs(inputs);
    currentAverageSpeed = speedAverage.addValue(inputs.shooterSpeed.baseUnitMagnitude());
    Logger.recordOutput("Shooter/AverageSpeed", currentAverageSpeed);

    switch (shooterState) {
      case Inactive:
        io.stopIndexing();
        break;
      case Revving:
        if (isSpunUp()) {
          this.shooterState = ShooterState.Shooting;
          io.startIndexing();
        } else {
          io.stopIndexing();
        }
        break;
      case Shooting:
        if (!isSpunUp()) {
          this.shooterState = ShooterState.Revving;
        }
        break;
      default:
        break;
    }

    if (inputs.isFuelInShooter && !wasFuelInShooter) {
      shotCount += 1;
    }
    wasFuelInShooter = inputs.isFuelInShooter;

    Logger.processInputs("Shooter", inputs);
  }

  public AngularVelocity getShooterSpeed() {
    return inputs.shooterSpeed;
  }

  public boolean isSpunUp() {
    return currentAverageSpeed >= Constants.ShooterConstants.minLaunchSpeed.baseUnitMagnitude()
        && speedAverage.getStandardDeviation(currentAverageSpeed) <= 12.;
  }

  public void setPitch(Rotation2d pitch) {
    io.setPitch(pitch);
  }

  public void setYaw(Rotation2d yaw) {
    double yawRadians =
        Math.max(
            Math.min(yaw.getRadians(), Constants.ShooterConstants.maxTurretYaw.getRadians()),
            Constants.ShooterConstants.minTurretYaw.getRadians());

    io.setYaw(new Rotation2d(yawRadians));
  }
}
