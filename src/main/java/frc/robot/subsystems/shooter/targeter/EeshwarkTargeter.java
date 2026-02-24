package frc.robot.subsystems.shooter.targeter;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants;
import frc.robot.subsystems.shooter.targeter.TargetingResult.TargetingResult2d;
import frc.robot.subsystems.shooter.targeter.TargetingResult.TargetingResult3d;
import java.util.Optional;

public class EeshwarkTargeter implements KinematicTargeter {

  @Override
  public Optional<TargetingResult3d> getShooterTargeting(TargetingData targetingData) {
    double projectileVelocity = targetingData.projectileVelocity().in(MetersPerSecond);

    double distance = targetingData.target().getNorm();
    Translation2d directionToTarget = targetingData.target().div(distance);
    TargetingResult2d staticVelocity =
        this.getShooterTargetingWithoutVelocity(
            distance,
            targetingData.targetHeight().in(Meters)
                - Constants.ShooterConstants.positionOnRobot.getZ(),
            projectileVelocity);
    double staticHorizontalVelocity = Math.cos(staticVelocity.pitchRadians()) * projectileVelocity;

    Translation2d staticShotVelocity = directionToTarget.times(staticHorizontalVelocity);

    Translation2d shotVector = staticShotVelocity.minus(targetingData.robotVelocity());

    if (shotVector.getSquaredNorm() == 0
        || Double.isNaN(shotVector.getX())
        || Double.isNaN(shotVector.getY())) {
      return Optional.empty();
    }

    Rotation2d shotYaw = shotVector.getAngle();
    double fieldRelativeYaw = shotYaw.getRadians();
    double requiredHorizontalVelocity = shotVector.getNorm();
    double requiredPitch = Math.acos((requiredHorizontalVelocity / projectileVelocity));

    return Optional.of(
        new TargetingResult3d(
            requiredPitch, fieldRelativeYaw, distance / requiredHorizontalVelocity));
  }
}
