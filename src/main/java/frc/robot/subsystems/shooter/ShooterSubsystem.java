package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class ShooterSubsystem extends SubsystemBase {

  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();
  private final Supplier<Translation2d> robotPositionSupplier;
  private final BooleanSupplier shouldTargetHub;

  @AutoLogOutput private boolean isShooting = false;
  @AutoLogOutput private boolean isIndexing = false;

  public ShooterSubsystem(
      ShooterIO io, Supplier<Pose2d> robotPositionSupplier, BooleanSupplier shouldTargetHub) {
    this.io = io;
    this.robotPositionSupplier = () -> robotPositionSupplier.get().getTranslation();
    this.shouldTargetHub = shouldTargetHub;
  }

  public void beginShooting() {
    isShooting = true;
    io.spinUpShooter();
  }

  public void endShooting() {
    isShooting = true;
    io.stopShooter();
    // io.endShooting();
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);

    if (shouldTargetHub.getAsBoolean()) {
      // Translation2d targetingDelta = Constants.Field.hubTarget.toTranslation2d()
      // .minus(robotPositionSupplier.get().getTranslation().toTranslation2d());
      //
      Translation3d hubTarget = Constants.Field.hubTarget;
      Translation2d shooter =
          robotPositionSupplier
              .get()
              .plus(Constants.ShooterConstants.positionOnRobot.toTranslation2d());

      double x = hubTarget.toTranslation2d().getDistance(shooter); // target distance in meters
      double y = hubTarget.getZ() - Constants.ShooterConstants.positionOnRobot.getZ(); // target
      // height in
      // meters

      double g = 9.81;
      double v_squared = Math.pow(Constants.ShooterConstants.launchSpeed.in(MetersPerSecond), 2);

      // https://en.wikipedia.org/wiki/Projectile_motion#Angle_%CE%B8_required_to_hit_coordinate_(x,_y)
      double pitchRadians =
          Math.atan2(
              v_squared
                  + Math.sqrt(
                      Math.pow(v_squared, 2) - g * (g * Math.pow(x, 2) + 2 * y * v_squared)),
              g * x);
      Logger.recordOutput("Shooter/TargetingHeight", y);
      Logger.recordOutput("Shooter/TargetingVelocitySquared", v_squared);
      Logger.recordOutput("Shooter/TargetingDistance", x);
      Logger.recordOutput("Shooter/TargetingPitch", pitchRadians);

      io.setPitch(new Rotation2d(pitchRadians));
    }

    if (isShooting
        && inputs.shooterSpeed.baseUnitMagnitude()
            >= Constants.ShooterConstants.minLaunchSpeed.baseUnitMagnitude()) {
      io.startIndexing();
      isIndexing = true;

    } else {
      io.stopIndexing();
      isIndexing = false;
    }
    Logger.processInputs("Shooter", inputs);
  }
}
