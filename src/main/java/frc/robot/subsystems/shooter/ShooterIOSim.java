package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants;
import java.util.function.Supplier;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnFly;

public class ShooterIOSim implements ShooterIO {

  private Rotation2d pitch = new Rotation2d(Math.PI / 4.);
  private Rotation2d yaw = new Rotation2d();

  private final Supplier<Translation2d> robotPositionSupplier;
  private final Supplier<ChassisSpeeds> chassisSpeedsSupplier;

  public ShooterIOSim(
      Supplier<Pose2d> robotPositionSupplier, Supplier<ChassisSpeeds> chassisSpeedsSupplier) {
    this.robotPositionSupplier = () -> robotPositionSupplier.get().getTranslation();
    this.chassisSpeedsSupplier = chassisSpeedsSupplier;
  }

  public void updateInputs(ShooterIOInputs inputs) {
    inputs.currentPitch = pitch;
  }

  public void setPitch(Rotation2d pitch) {
    this.pitch = pitch;
  }

  public void setYaw(Rotation2d yaw) {
    this.yaw = yaw;
  }

  public void shoot() {
    System.out.println("firing fuel");
    RebuiltFuelOnFly fuel =
        new RebuiltFuelOnFly(
            robotPositionSupplier.get(),
            Constants.ShooterConstants.positionOnRobot.toTranslation2d(),
            chassisSpeedsSupplier.get(),
            yaw,
            Constants.ShooterConstants.positionOnRobot.getMeasureZ(),
            Constants.ShooterConstants.launchSpeed,
            pitch.getMeasure());

    fuel
        // Set the target center to the Crescendo Speaker of the current alliance
        .withTargetPosition(() -> new Translation3d(0.25, 5.56, 2.3))
        // Set the tolerance: x: ±0.5m, y: ±1.2m, z: ±0.3m (this is the size of the
        // speaker's "mouth")
        .withTargetTolerance(new Translation3d(0.5, 1.2, 0.3))
        // Set a callback to run when the note hits the target
        .withHitTargetCallBack(() -> System.out.println("Hit speaker, +2 points!"));

    SimulatedArena.getInstance().addGamePieceProjectile(fuel);
  }
}
