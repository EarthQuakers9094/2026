package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Radians;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class ShooterIOReal implements ShooterIO {

  private final TalonFX flywheelLeadMotor =
      new TalonFX(Constants.ShooterConstants.motor1Id, Constants.shooterCANBus);
  private final TalonFX flywheelFollowerMotor =
      new TalonFX(Constants.ShooterConstants.motor2Id, Constants.shooterCANBus);

  private final TalonFX turretPivot =
      new TalonFX(Constants.ShooterConstants.turretMotorId, Constants.shooterCANBus);
  private final TalonFX hoodPivot =
      new TalonFX(Constants.ShooterConstants.hoodMotorId, Constants.shooterCANBus);

  private final DigitalInput shooterBeamBrake =
      new DigitalInput(Constants.ShooterConstants.shooterBeamBrakePort);

  private final InterpolatingDoubleTreeMap flywheelVelocityToLinearVelocity =
      new InterpolatingDoubleTreeMap();
  private final InterpolatingDoubleTreeMap launchAngleToHoodAngle =
      new InterpolatingDoubleTreeMap();

  private final TrapezoidProfile hoodTrapezoidProfile =
      new TrapezoidProfile(Constants.ShooterConstants.hoodConstraints);
  private TrapezoidProfile.State hoodState = new TrapezoidProfile.State(0, 0);
  private TrapezoidProfile.State hoodSetpoint = new TrapezoidProfile.State(0, 0);

  private final TrapezoidProfile turretTrapezoidProfile =
      new TrapezoidProfile(Constants.ShooterConstants.turretConstraints);
  private TrapezoidProfile.State turretState = new TrapezoidProfile.State(0, 0);
  private TrapezoidProfile.State turretSetpoint = new TrapezoidProfile.State(0, 0);

  private double lastFlywheelVelocitySetpoint = 0;
  private double lastHoodSetpoint = 0;

  public ShooterIOReal() {
    flywheelLeadMotor
        .getConfigurator()
        .apply(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive));
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

    hoodPivot
        .getConfigurator()
        .apply(
            new Slot0Configs()
                .withKP(Constants.ShooterConstants.hoodKP)
                .withKI(Constants.ShooterConstants.hoodKI)
                .withKD(Constants.ShooterConstants.hoodKD)
                .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign)
                .withKS(Constants.ShooterConstants.hoodKS));
    // hoodPivot.get

    FeedbackConfigs hoodConversionFactor = new FeedbackConfigs();
    hoodConversionFactor.SensorToMechanismRatio = Constants.ShooterConstants.hoodConversionFactor;
    hoodPivot.getConfigurator().apply(hoodConversionFactor);
    hoodPivot
        .getConfigurator()
        .apply(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive));
    // hoodPivot
    //     .getConfigurator()
    //     .apply(
    //         new
    // CurrentLimitsConfigs().withSupplyCurrentLimit(50).withSupplyCurrentLowerLimit(40));
    hoodPivot.setPosition(0);

    // 0.0222589403665
    FeedbackConfigs turretConversionFactor = new FeedbackConfigs();
    turretConversionFactor.SensorToMechanismRatio =
        Constants.ShooterConstants.turretConversionFactor;
    turretPivot.getConfigurator().apply(turretConversionFactor);
    turretPivot
        .getConfigurator()
        .apply(
            new Slot0Configs()
                .withKP(Constants.ShooterConstants.turretKP)
                .withKI(Constants.ShooterConstants.turretKI)
                .withKD(Constants.ShooterConstants.turretKD));
    turretPivot
        .getConfigurator()
        .apply(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive));

    turretPivot.setPosition(Constants.ShooterConstants.turretZeroYaw.getMeasure());

    // TODO: Fill out on actual robot. See:
    // https://www.desmos.com/calculator/btzzpyy6r2

    flywheelVelocityToLinearVelocity.put(0., 0.);
    launchAngleToHoodAngle.put(0., 0.);
  }

  public void updateInputs(ShooterIOInputs inputs) {
    inputs.shooterSpeed = flywheelLeadMotor.getRotorVelocity().getValue();
    inputs.isFuelInShooter = shooterBeamBrake.get();
    inputs.yaw = turretPivot.getPosition().getValue();
    inputs.hoodPosition = hoodPivot.getPosition().getValueAsDouble();

    if (lastFlywheelVelocitySetpoint < 1.0 && inputs.shooterSpeed.in(RPM) < 500.0) {
      flywheelLeadMotor.stopMotor();
    }
    Logger.recordOutput(
        "Shooter/LastSetpointRadPerSec", lastFlywheelVelocitySetpoint * (2 * Math.PI) / 60);

    hoodState = hoodTrapezoidProfile.calculate(0.02, hoodState, hoodSetpoint);
    Logger.recordOutput("Shooter/LastSmoothHood", hoodState.position);

    hoodPivot.setControl(new PositionVoltage(hoodState.position).withSlot(0));

    turretState = turretTrapezoidProfile.calculate(0.02, turretState, turretSetpoint);
    Logger.recordOutput("Shooter/LastSmoothTurret", turretState.position);

    // Logger.recordOutput("Shooter/HoodCurrent", hoodPivot.getCurr);

    turretPivot.setControl(new PositionVoltage(Radians.of(turretState.position)).withSlot(0));
  }

  public void setHoodAngle(double pitch) {
    // double requiredHoodAngle = launchAngleToHoodAngle.get(pitch.getRadians());
    double clampedPitch = Math.max(Math.min(2.407227, pitch), 0.1);

    Logger.recordOutput("Shooter/LastHoodSetpoint", clampedPitch);

    hoodSetpoint = new TrapezoidProfile.State(clampedPitch, 0);
  }

  public void setHoodSpeed(double speed) {
    hoodPivot.set(speed);
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
    Logger.recordOutput("Shooter/YawSetpointRadians", yawRadians);
    turretSetpoint = new TrapezoidProfile.State(yawRadians, 0);
  }

  public void setVelocitySetpoint(AngularVelocity speed) {
    this.lastFlywheelVelocitySetpoint = speed.in(RPM);
    flywheelLeadMotor.setControl(new VelocityVoltage(speed));
  }
}
