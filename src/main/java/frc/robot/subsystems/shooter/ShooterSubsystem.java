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

  private AngularVelocity targetSpeed = Constants.ShooterConstants.launchSpeed;
  // private double speedSetpointRPM = 0.0;

  private boolean wasFuelInShooter = false;
  @AutoLogOutput public int shotCount = 0;

  public enum ShooterState {
    Inactive,
    Revving,
    Shooting
  }

  @AutoLogOutput private ShooterState shooterState = ShooterState.Inactive;
  private boolean shouldShootWhenReady = false;

  public ShooterSubsystem(ShooterIO io) {
    Logger.recordOutput("ShooterSubsystem/ShootingIsHappening", 0);

    this.io = io;
  }

  private void setSpeedSetpoint(AngularVelocity speed) {
    io.setVelocitySetpoint(speed);
    // speedSetpointRPM = speed.in(RPM);
    Logger.recordOutput("Shooter/SpeedSetpointRadPerSec", speed.in(RadiansPerSecond));
  }

  public void setTargetAngularVelocity(AngularVelocity speed) {
    if (shooterState != ShooterState.Inactive) {
      setSpeedSetpoint(speed);
    }
    this.targetSpeed = speed;
  }

  public AngularVelocity getTargetAngularVelocity() {
    return this.targetSpeed;
  }

  // public void startShooter() {
  //   if (shooterState == ShooterState.Inactive) {
  //     Logger.recordOutput("ShooterSubsystem/ShootingIsHappening", 1);

  //     shooterState = ShooterState.Revving;
  //     setSpeedSetpoint(Constants.ShooterConstants.launchSpeed);
  //   }
  // }

  // public void stopShooting() {
  //   switch (shooterState) {
  //     case Shooting:
  //       shooterState = ShooterState.Revving;
  //     default:
  //       break;

  //   }
  // }

  // public void endShooter() {
  //   Logger.recordOutput("ShooterSubsystem/ShootingIsHappening", -1);
  //   shooterState = ShooterState.Inactive;
  //   setSpeedSetpoint(RPM.of(0.));
  // }

  public void revShooter() {
    if (shooterState == ShooterState.Inactive) {
      Logger.recordOutput("ShooterSubsystem/ShootingIsHappening", 1);
      shooterState = ShooterState.Revving;
    }
  }

  public void stopShooter() {
    shooterState = ShooterState.Inactive;
  }

  public void setReadyToShoot(boolean readyToShoot) {
    this.shouldShootWhenReady = readyToShoot;
  }

  public boolean isActivelyShooting() {
    return shooterState == ShooterState.Shooting;
  }

  @Override
  public void periodic() {

    io.updateInputs(inputs);
    currentAverageSpeed = speedAverage.addValue(inputs.shooterSpeed.in(RadiansPerSecond));
    Logger.recordOutput("Shooter/AverageSpeedRadPerSec", currentAverageSpeed);
    Logger.recordOutput("Shooter/State", shooterState);
    Logger.recordOutput(
        "Shooter/LaunchAngleDegrees",
        hoodAngleToLaunchAngle(inputs.hoodPosition) * (180 / Math.PI));

    switch (shooterState) {
      case Inactive:
        setSpeedSetpoint(RPM.of(0.0));
        break;
      case Revving:
        setSpeedSetpoint(targetSpeed);
        if (isSpunUp() && shouldShootWhenReady) {
          this.shooterState = ShooterState.Shooting;
        }
        break;
      case Shooting:
        if (!isSpunUp() || !shouldShootWhenReady) {
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

  @AutoLogOutput
  public boolean isSpunUp() {
    return isAboveMinLaunchSpeed() && isSpeedStable();
  }

  @AutoLogOutput
  public boolean isSpeedStable() {
    return speedAverage.getStandardDeviation(currentAverageSpeed) <= 12.;
  }

  @AutoLogOutput
  private boolean isAboveMinLaunchSpeed() {
    return currentAverageSpeed >= (targetSpeed.in(RadiansPerSecond) * 0.9);
  }

  public void setPitch(Rotation2d pitch) {

    double pitchRadians =
        Math.max(
            Math.min(pitch.getRadians(), Constants.ShooterConstants.maxLaunchAngle.getRadians()),
            Constants.ShooterConstants.minLaunchAngle.getRadians());
    Logger.recordOutput("Shooter/ClampedLaunchAngle", pitchRadians);

    io.setHoodAngle(launchAngleToHoodAngle(pitchRadians));
  }

  public void setHoodAngle(double hoodAngle) {
    io.setHoodAngle(hoodAngle);
  }

  public void setYaw(Rotation2d yaw) {
    double yawRadians =
        Math.max(
            Math.min(yaw.getRadians(), Constants.ShooterConstants.maxTurretYaw.getRadians()),
            Constants.ShooterConstants.minTurretYaw.getRadians());

    io.setYaw(new Rotation2d(yawRadians));
  }

  public void retractHood() {
    io.retractHood();
  }

  private static double launchAngleToHoodAngle(double launchAngleRad) {
    return 0.940536 * Math.sin(5.98518 * launchAngleRad + 1.78146) + 1.46627;
  }

  private static double hoodAngleToLaunchAngle(double hoodAngle) {
    return 9.59432 * Math.pow(hoodAngle, 2) + 27.24729 * hoodAngle - 19.96681;
    // return 0.940536 * Math.sin(5.98518 * hoodAngle + 1.78146) + 1.46627;
  }

  public static double shooterSpeedToVelocity(double shooterSpeedRadPerSec) {
    return 0.0209048 * shooterSpeedRadPerSec + 0.742784;
  }

  public static AngularVelocity velocityToShooterSpeed(double velocityMPS) {
    return RadiansPerSecond.of(-47.8359 * (0.742784 - velocityMPS));
  }

  public static AngularVelocity getIdealShooterSpeed(double distanceToTarget) {
    if (distanceToTarget > 1.0) {
      return RPM.of(3500);
    } else {
      return RPM.of(2500);
    }
}
}
