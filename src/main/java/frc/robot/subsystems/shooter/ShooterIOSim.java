package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RPM;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import frc.robot.Constants;
import java.util.function.Supplier;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnFly;

public class ShooterIOSim implements ShooterIO {

  private Rotation2d pitch = new Rotation2d(Math.PI / 4.);
  private Rotation2d yaw = new Rotation2d();

  private DCMotor flywheelMotor = DCMotor.getNeoVortex(1);

  private final SparkFlex flex =
      new SparkFlex(Constants.ShooterConstants.motorId, MotorType.kBrushless);
  private final SparkFlexSim flexSim = new SparkFlexSim(flex, flywheelMotor);

  private FlywheelSim flywheelSim =
      new FlywheelSim(
          LinearSystemId.createFlywheelSystem(
              DCMotor.getNeoVortex(1),
              Constants.ShooterConstants.flywheelMOI,
              Constants.ShooterConstants.flywheelGearing),
          flywheelMotor);

  private final Supplier<Translation2d> robotPositionSupplier;
  private final Supplier<ChassisSpeeds> chassisSpeedsSupplier;

  private boolean isIndexing = false;
  private double lastShotFuelS = Timer.getFPGATimestamp();

  public ShooterIOSim(
      Supplier<Pose2d> robotPositionSupplier, Supplier<ChassisSpeeds> chassisSpeedsSupplier) {
    this.robotPositionSupplier = () -> robotPositionSupplier.get().getTranslation();
    this.chassisSpeedsSupplier = chassisSpeedsSupplier;

    flex.configure(
        new SparkFlexConfig().apply(new ClosedLoopConfig().pid(0.0025, 0.0, 0.0)),
        ResetMode.kResetSafeParameters,
        PersistMode.kNoPersistParameters);
  }

  public void updateInputs(ShooterIOInputs inputs) {

    // Logger.recordOutput("Shooter/SetpointRPM",
    // flex.getClosedLoopController().getSetpoint());
    // Logger.recordOutput(
    // "Shooter/StateRPM",
    // RPM.of(flex.getEncoder().getVelocity()).baseUnitMagnitude());

    flywheelSim.setInput(flexSim.getAppliedOutput() * RoboRioSim.getVInVoltage());

    flywheelSim.update(0.02);

    flexSim.iterate(flywheelSim.getAngularVelocityRPM(), RoboRioSim.getVInVoltage(), 0.02);

    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(flywheelSim.getCurrentDrawAmps()));

    inputs.currentPitch = pitch;
    inputs.shooterSpeed = flywheelSim.getAngularVelocity();

    double newTime = Timer.getFPGATimestamp();
    double delta = newTime - lastShotFuelS;

    if (isIndexing && delta >= 0.1) {
      lastShotFuelS = newTime;
      RebuiltFuelOnFly fuel =
          new RebuiltFuelOnFly(
              robotPositionSupplier.get(),
              Constants.ShooterConstants.positionOnRobot.toTranslation2d(),
              chassisSpeedsSupplier.get(),
              yaw,
              Constants.ShooterConstants.positionOnRobot.getMeasureZ(),
              MetersPerSecond.of(
                  flywheelSim.getAngularVelocityRadPerSec()
                      * Constants.ShooterConstants.flywheelDiameter.in(Meters)),
              pitch.getMeasure());
      SimulatedArena.getInstance().addGamePieceProjectile(fuel);
    }
  }

  public void setPitch(Rotation2d pitch) {
    this.pitch = pitch;
  }

  public void setYaw(Rotation2d yaw) {
    this.yaw = yaw;
  }

  // public void shoot() {
  // System.out.println("firing fuel");
  // RebuiltFuelOnFly fuel = new RebuiltFuelOnFly(
  // robotPositionSupplier.get(),
  // Constants.ShooterConstants.positionOnRobot.toTranslation2d(),
  // chassisSpeedsSupplier.get(),
  // yaw,
  // Constants.ShooterConstants.positionOnRobot.getMeasureZ(),
  // Constants.ShooterConstants.launchSpeed,
  // pitch.getMeasure());
  //
  // fuel
  // // Set the target center to the Crescendo Speaker of the current alliance
  // .withTargetPosition(() -> new Translation3d(0.25, 5.56, 2.3))
  // // Set the tolerance: x: ±0.5m, y: ±1.2m, z: ±0.3m (this is the size of the
  // // speaker's "mouth")
  // .withTargetTolerance(new Translation3d(0.5, 1.2, 0.3))
  // // Set a callback to run when the note hits the target
  // .withHitTargetCallBack(() -> System.out.println("Hit speaker, +2 points!"));
  //
  // SimulatedArena.getInstance().addGamePieceProjectile(fuel);
  // }

  public void setVelocitySetpoint(AngularVelocity speed) {
    flex.getClosedLoopController().setSetpoint(speed.in(RPM), ControlType.kVelocity);
  }
  ;

  public void stopShooter() {
    flex.getClosedLoopController().setSetpoint(0.0, ControlType.kVelocity);
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
