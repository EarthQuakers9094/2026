package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import java.util.function.Supplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.MovingAverage;

public class ShooterSubsystem extends SubsystemBase {

  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

  private final MovingAverage speedAverage = new MovingAverage(40);
  private double currentAverageSpeed;

  private AngularVelocity targetSpeed = Constants.ShooterConstants.launchSpeed;

  private final LoggedMechanism2d hoodMechanism = new LoggedMechanism2d(3, 3);
  private final LoggedMechanismRoot2d hoodMechanismRoot =
      hoodMechanism.getRoot("hoodRoot", 1.5, .5);
  private final LoggedMechanismLigament2d hoodVisualization =
      hoodMechanismRoot.append(
          new LoggedMechanismLigament2d("hood", Inches.of(0.0).in(Meters), 90));

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

  private final Supplier<Pose2d> robotPositionSupplier;

  public ShooterSubsystem(ShooterIO io, Supplier<Pose2d> robotPositionSupplier) {
    Logger.recordOutput("ShooterSubsystem/ShootingIsHappening", 0);

    this.robotPositionSupplier = robotPositionSupplier;

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
  // if (shooterState == ShooterState.Inactive) {
  // Logger.recordOutput("ShooterSubsystem/ShootingIsHappening", 1);

  // shooterState = ShooterState.Revving;
  // setSpeedSetpoint(Constants.ShooterConstants.launchSpeed);
  // }
  // }

  // public void stopShooting() {
  // switch (shooterState) {
  // case Shooting:
  // shooterState = ShooterState.Revving;
  // default:
  // break;

  // }
  // }

  // public void endShooter() {
  // Logger.recordOutput("ShooterSubsystem/ShootingIsHappening", -1);
  // shooterState = ShooterState.Inactive;
  // setSpeedSetpoint(RPM.of(0.));
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

    Logger.recordOutput(
        "TurretVisualization",
        robotPositionSupplier
            .get()
            .plus(
                new Transform2d(
                    new Translation2d(1.0, new Rotation2d(inputs.yaw)), new Rotation2d())));
    io.updateInputs(inputs);
    currentAverageSpeed = speedAverage.addValue(inputs.shooterSpeed.in(RadiansPerSecond));

    hoodVisualization.setLength(Inches.of(inputs.hoodPosition));

    Logger.recordOutput("Shooter/HoodVisualization", hoodMechanism);
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
    return true;
    // return speedAverage.getStandardDeviation(currentAverageSpeed) <= 12.;
  }

  @AutoLogOutput
  private boolean isAboveMinLaunchSpeed() {
    return currentAverageSpeed >= (targetSpeed.in(RadiansPerSecond) * 0.85);
  }

  public void setPitch(Rotation2d pitch) {

    double pitchRadians =
        Math.max(
            Math.min(pitch.getRadians(), Constants.ShooterConstants.maxLaunchAngle.getRadians()),
            Constants.ShooterConstants.minLaunchAngle.getRadians());
    Logger.recordOutput("Shooter/ClampedLaunchAngle", pitchRadians);
    Logger.recordOutput("Setting launch angle", Timer.getFPGATimestamp());
    setHoodAngle(launchAngleToHoodAngle(pitchRadians));
  }

  public void setHoodAngle(double hoodAngle) {
    Logger.recordOutput("Setting hood angle", Timer.getFPGATimestamp());

    // if (hoodAngle != 0) {
    //   System.out.println(DriverStation.getMatchTime());
    // }

    io.setHoodAngle(hoodAngle);
  }

  public void setYaw(Rotation2d yaw) {
    double yawRadians =
        Math.max(
            Math.min(yaw.getRadians(), Constants.ShooterConstants.maxTurretYaw.getRadians()),
            Constants.ShooterConstants.minTurretYaw.getRadians());

    io.setYaw(new Rotation2d(yawRadians));
  }

  public Angle getYaw() {
    return inputs.yaw;
  }

  public void retractHood() {
    io.retractHood();
  }

  public void runHoodDown() {
    io.setHoodSpeed(-0.1);
  }

  public void zeroHood() {
    io.zeroHood();
  }

  public void stopHood() {
    io.setHoodSpeed(0.0);
  }

  public double getHoodCurrent() {
    return inputs.hoodCurrent;
  }

  public static double launchAngleToHoodAngle(double launchAngleRad) {

    return -4.84013 * launchAngleRad
        + 7.57876; // 0.940536 * Math.sin(5.98518 * launchAngleRad + 1.78146) + 1.46627;
  }

  public static double hoodAngleToLaunchAngle(double hoodAngle) {

    return 0.206606 * (7.57876 - hoodAngle); // return 9.59432 * Math.pow(hoodAngle, 2) + 27.24729 *
    // hoodAngle - 19.96681;
    // return 0.940536 * Math.sin(5.98518 * hoodAngle + 1.78146) + 1.46627;
  }

  public static double shooterSpeedToVelocity(double shooterSpeedRadPerSec) {
    return 0.0209048 * shooterSpeedRadPerSec + 0.742784;
  }

  public static AngularVelocity velocityToShooterSpeed(double velocityMPS) {
    return RadiansPerSecond.of(-47.8359 * (0.742784 - velocityMPS));
  }



  public static double getIdealPitch(double distanceToTarget) {
    return -0.180371 * distanceToTarget + 1.6617; // -0.128837 * distanceToTarget + 1.58586;
  }
  // if (distanceToTarget <= 2.0) {
  // return Math.PI / 2;
  // } else if (distanceToTarget <= 4.0) {
  // // Logger.recordOutput("Shooter/DistanceToTarget", "near");
  // return 9 * Math.PI / 20;
  // }
  // return 0;
  // // TODO Auto-generated method stub
  // // throw new UnsupportedOperationException("Unimplemented method
  // 'getIdealPitch'");
  // }

  public double getHoodAngle() {
    // TODO Auto-generated method stub
return inputs.hoodPosition;  }
}
