package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Radians;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;

public class ShooterIOReal implements ShooterIO {

  private final TalonFX flywheelLeadMotor = new TalonFX(Constants.ShooterConstants.motor1Id);
  private final TalonFX flywheelFollowerMotor = new TalonFX(Constants.ShooterConstants.motor2Id);

  private final TalonFX turretPivot = new TalonFX(Constants.ShooterConstants.turretMotorId);
  private final TalonFX hoodPivot = new TalonFX(Constants.ShooterConstants.hoodMotorId);

  private final DigitalInput shooterBeamBrake =
      new DigitalInput(Constants.ShooterConstants.shooterBeamBrakePort);
  private final DigitalInput turretYawLimit =
      new DigitalInput(Constants.ShooterConstants.turretYawLimitPort);

  private final TalonFX indexerMotor = new TalonFX(Constants.IndexerConstants.indexerMotorId);

  private final InterpolatingDoubleTreeMap flywheelVelocityToLinearVelocity =
      new InterpolatingDoubleTreeMap();
  private final InterpolatingDoubleTreeMap launchAngleToHoodAngle =
      new InterpolatingDoubleTreeMap();

  public ShooterIOReal() {
    flywheelFollowerMotor.setControl(
        new Follower(flywheelLeadMotor.getDeviceID(), MotorAlignmentValue.Opposed));
    flywheelLeadMotor
        .getConfigurator()
        .apply(
            new Slot0Configs()
                .withKP(Constants.ShooterConstants.flywheelKP)
                .withKI(Constants.ShooterConstants.flywheelKI)
                .withKD(Constants.ShooterConstants.flywheelKD)
                .withKV(Constants.ShooterConstants.flywheelKV));

    TalonFXConfigurator hoodConfigurator = hoodPivot.getConfigurator();
    hoodConfigurator.apply(
        new Slot0Configs()
            .withKP(Constants.ShooterConstants.hoodKP)
            .withKI(Constants.ShooterConstants.hoodKI)
            .withKD(Constants.ShooterConstants.hoodKD));
    hoodPivot.setPosition(Constants.ShooterConstants.hoodStartPosition);

    turretPivot
        .getConfigurator()
        .apply(
            new Slot0Configs()
                .withKP(Constants.ShooterConstants.turretKP)
                .withKI(Constants.ShooterConstants.turretKI)
                .withKD(Constants.ShooterConstants.turretKD));

    // TODO: Fill out on actual robot. See:
    // https://www.desmos.com/calculator/btzzpyy6r2

    // flywheelVelocityToLinearVelocity.put(, );
    // launchAngleToHoodAngle.put(, );

  }

  public void updateInputs(ShooterIOInputs inputs) {
    inputs.shooterSpeed = flywheelLeadMotor.getRotorVelocity().getValue();
    inputs.isFuelInShooter = shooterBeamBrake.get();
    inputs.isAtTurretLimit = turretYawLimit.get();
  }

  public void setPitch(Rotation2d pitch) {
    double requiredHoodAngle = launchAngleToHoodAngle.get(pitch.getRadians());

    hoodPivot.setControl(new PositionVoltage(requiredHoodAngle).withSlot(0));
  }

  public void setYaw(Rotation2d yaw) {
    double yawRadians = yaw.getRadians();
    double maxRadians = Constants.ShooterConstants.maxTurretYaw.getRadians();
    double minRadians = Constants.ShooterConstants.minTurretYaw.getRadians();
    if (yawRadians > maxRadians || yawRadians < minRadians) {
      DriverStation.reportWarning(
          "Yaw: " + yawRadians + " is outside of the turret's safe range", true);
      yawRadians = Math.max(Math.min(yawRadians, maxRadians), minRadians);
    }
    turretPivot.setControl(new PositionVoltage(Radians.of(yawRadians)).withSlot(0));
  }

  public void setVelocitySetpoint(AngularVelocity speed) {
    flywheelLeadMotor.setControl(new VelocityVoltage(speed));
  }

  public void startIndexing() {
    indexerMotor.setControl(new DutyCycleOut(1.0));
  }

  public void stopIndexing() {
    indexerMotor.setControl(new DutyCycleOut(0.0));
  }

  public void setMeasuredTurretYaw(Angle newYaw) {
    turretPivot.setPosition(newYaw);
  }

  public void slowlyMoveTowardsLimit() {
    turretPivot.set(Constants.ShooterConstants.crawlSpeed);
  }
}
