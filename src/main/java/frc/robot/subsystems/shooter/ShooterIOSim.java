package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.sim.TalonFXSimState.MotorType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import frc.robot.Constants;
import java.util.function.Supplier;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnFly;
import org.littletonrobotics.junction.Logger;

public class ShooterIOSim implements ShooterIO {

  private Rotation2d pitch = new Rotation2d(Math.PI / 4.);
  private Rotation2d yaw = new Rotation2d();

  private DCMotor flywheelMotor = DCMotor.getKrakenX60(1);

  private final TalonFX flywheelLeadMotor = new TalonFX(Constants.ShooterConstants.motor1Id);
  private final TalonFX flywheelFollowerMotor = new TalonFX(Constants.ShooterConstants.motor2Id);

  private final TalonFXSimState leadMotorSimState;
  private final TalonFXSimState followerMotorSimState;

  private FlywheelSim flywheelSim =
      new FlywheelSim(
          LinearSystemId.createFlywheelSystem(
              flywheelMotor,
              Constants.ShooterConstants.flywheelMOI,
              Constants.ShooterConstants.flywheelGearing),
          flywheelMotor);

  private final Supplier<Pose2d> robotPositionSupplier;
  private final Supplier<ChassisSpeeds> chassisSpeedsSupplier;

  // private final Mechanism2d turretVisulization = new Mechanism2d(3, 3);
  // private final MechanismLigament2d turretLigament;

  private boolean isIndexing = false;
  private double lastShotFuelS = Timer.getFPGATimestamp();

  public ShooterIOSim(
      Supplier<Pose2d> robotPositionSupplier, Supplier<ChassisSpeeds> chassisSpeedsSupplier) {
    this.robotPositionSupplier = robotPositionSupplier;
    this.chassisSpeedsSupplier = chassisSpeedsSupplier;

    flywheelFollowerMotor.setControl(
        new Follower(flywheelLeadMotor.getDeviceID(), MotorAlignmentValue.Opposed));

    flywheelLeadMotor
        .getConfigurator()
        .apply(new Slot0Configs().withKP(0.02).withKI(0.0).withKD(0.0).withKV(0.06));

    this.leadMotorSimState = flywheelLeadMotor.getSimState();
    this.followerMotorSimState = flywheelFollowerMotor.getSimState();

    leadMotorSimState.Orientation = ChassisReference.CounterClockwise_Positive;
    followerMotorSimState.Orientation = ChassisReference.Clockwise_Positive;

    leadMotorSimState.setMotorType(MotorType.KrakenX60);
  }

  public void updateInputs(ShooterIOInputs inputs) {
    Logger.recordOutput(
        "TurretVisualization",
        robotPositionSupplier
            .get()
            .plus(new Transform2d(new Translation2d(1.0, yaw), new Rotation2d())));

    leadMotorSimState.setSupplyVoltage(RoboRioSim.getVInVoltage());
    followerMotorSimState.setSupplyVoltage(RoboRioSim.getVInVoltage());

    Voltage leadVoltage = leadMotorSimState.getMotorVoltageMeasure();
    Voltage followerVoltage = followerMotorSimState.getMotorVoltageMeasure();

    Logger.recordOutput("ShooterSubsystem/LeadVoltage", leadVoltage);

    flywheelSim.setInput(leadVoltage.in(Volts) + followerVoltage.in(Volts));

    flywheelSim.update(0.02);

    leadMotorSimState.setRotorVelocity(flywheelSim.getAngularVelocity());
    followerMotorSimState.setRotorVelocity(flywheelSim.getAngularVelocity());

    // leadMotorSimState.addRotorPosition(
    // flywheelSim.getAngularVelocity().in(RotationsPerSecond) * 0.02);
    // followerMotorSimState.setRotorVelocity(flywheelSim.getAngularVelocity());

    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(flywheelSim.getCurrentDrawAmps()));

    inputs.shooterSpeed = flywheelSim.getAngularVelocity();

    double newTime = Timer.getFPGATimestamp();
    double delta = newTime - lastShotFuelS;
    inputs.isFuelInShooter = false;
    if (isIndexing && delta >= 0.1) {
      inputs.isFuelInShooter = true;
      lastShotFuelS = newTime;
      RebuiltFuelOnFly fuel =
          new RebuiltFuelOnFly(
              robotPositionSupplier.get().getTranslation(),
              Constants.ShooterConstants.positionOnRobot.getTranslation().toTranslation2d(),
              chassisSpeedsSupplier.get(),
              yaw.plus(robotPositionSupplier.get().getRotation()),
              Constants.ShooterConstants.positionOnRobot.getMeasureZ(),
              MetersPerSecond.of(
                  flywheelSim.getAngularVelocityRadPerSec()
                      * Constants.ShooterConstants.flywheelDiameter.in(Meters)),
              pitch.getMeasure());
      SimulatedArena.getInstance().addGamePieceProjectile(fuel);
    }

    Translation2d robotPosition = robotPositionSupplier.get().getTranslation();

    drawTrajectory(
        new Translation3d(
            robotPosition.getX(),
            robotPosition.getY(),
            Constants.ShooterConstants.positionOnRobot.getZ()),
        chassisSpeedsSupplier.get(),
        yaw.plus(robotPositionSupplier.get().getRotation()),
        pitch.getRadians(),
        inputs.shooterSpeed.in(RadiansPerSecond)
            * Constants.ShooterConstants.flywheelDiameter.in(Meters));
  }

  public void drawTrajectory(
      Translation3d startPosition,
      ChassisSpeeds chassisSpeeds,
      Rotation2d yaw,
      double pitch,
      double launchVelocity) {

    Pose3d[] poses = new Pose3d[20];

    double vx = chassisSpeeds.vxMetersPerSecond + yaw.getCos() * launchVelocity * Math.cos(pitch);
    double vy = chassisSpeeds.vyMetersPerSecond + yaw.getSin() * launchVelocity * Math.cos(pitch);
    double vz = Math.sin(pitch) * launchVelocity;

    double x = startPosition.getX();
    double y = startPosition.getY();
    double z = startPosition.getZ();

    double dt = 0.1;

    for (int i = 0; i < 20; i++) {
      poses[i] = new Pose3d(x, y, z, new Rotation3d());
      vz -= 9.81 * dt;

      x += vx * dt;
      y += vy * dt;
      z += vz * dt;
    }

    Logger.recordOutput("Shooter Trajectory", poses);
  }

  public void setPitch(Rotation2d pitch) {
    this.pitch = pitch;
  }

  public void setYaw(Rotation2d yaw) {

    this.yaw = yaw;
  }

  public void setVelocitySetpoint(AngularVelocity speed) {
    flywheelLeadMotor.setControl(new VelocityVoltage(speed.in(RotationsPerSecond)).withSlot(0));
  }
  ;

  public void stopShooter() {
    flywheelLeadMotor.setControl(new VelocityVoltage(0.0).withSlot(0));
  }
  ;

  public void startIndexing() {
    isIndexing = true;
  }
  ;

  public void stopIndexing() {
    isIndexing = false;
  }
  ;
}
