package frc.robot.subsystems.shooter.targeter;

import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.shooter.targeter.TargetingResult.TargetingResult3d;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;

public class EeshwarkTargeter implements KinematicTargeter {

  private InterpolatingDoubleTreeMap distanceToRPM = new InterpolatingDoubleTreeMap();

  public EeshwarkTargeter() {
    distanceToRPM.put(4.776835, 3600d);
    distanceToRPM.put(4.305231, 3400d);
    distanceToRPM.put(3.943729, 3250d);

    distanceToRPM.put(3.63685, 3100d);

    distanceToRPM.put(2.878153, 2900d);

    distanceToRPM.put(2.652807, 2800d);

    distanceToRPM.put(2.200375, 2700d);

    distanceToRPM.put(1.918899, 2600d);
  }

  @Override
  public Optional<TargetingResult3d> getShooterTargeting(TargetingData targetingData) {
    // double projectileVelocity = targetingData.projectileVelocity().in(MetersPerSecond);
    double distance = targetingData.target().getNorm();
    Logger.recordOutput("DistancePassedToTargeter", distance);

    Translation2d directionToTarget = targetingData.target().div(distance);
    // TargetingResult2d staticVelocity =
    //     this.getShooterTargetingWithoutVelocity(
    //         distance,
    //         targetingData.targetHeight().in(Meters)
    //             - Constants.ShooterConstants.positionOnRobot.getZ(),
    //         projectileVelocity);
    double baseRPM = distanceToRPM.get(distance);
    double projectileVelocity = ShooterSubsystem.shooterSpeedToVelocity(baseRPM * (Math.PI / 30.));
    double staticHorizontalVelocity =
        Math.cos(targetingData.launchAngle().getRadians()) * projectileVelocity;

    Translation2d staticShotVelocity = directionToTarget.times(staticHorizontalVelocity);

    Translation2d shotVector = staticShotVelocity.plus(targetingData.robotVelocity());

    if (shotVector.getSquaredNorm() == 0
        || Double.isNaN(shotVector.getX())
        || Double.isNaN(shotVector.getY())) {
      return Optional.empty();
    }

    Rotation2d shotYaw = shotVector.getAngle();
    double fieldRelativeYaw = shotYaw.getRadians();
    double requiredHorizontalVelocity = shotVector.getNorm();
    double requiredVelocity = requiredHorizontalVelocity / targetingData.launchAngle().getCos();

    double requiredPitch = Math.acos((requiredHorizontalVelocity / projectileVelocity));

    return Optional.of(
        new TargetingResult3d(
            ShooterSubsystem.velocityToShooterSpeed(requiredVelocity).in(RPM),
            fieldRelativeYaw,
            distance / requiredHorizontalVelocity));
  }
}
