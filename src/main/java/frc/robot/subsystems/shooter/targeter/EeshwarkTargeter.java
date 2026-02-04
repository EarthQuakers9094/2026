package frc.robot.subsystems.shooter.targeter;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants;
import frc.robot.subsystems.shooter.targeter.TargetingResult.TargetingResult2d;
import frc.robot.subsystems.shooter.targeter.TargetingResult.TargetingResult3d;

public class EeshwarkTargeter implements KinematicTargeter {

  @Override
  public TargetingResult3d getShooterTargeting(TargetingData targetingData) {
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

    double fieldRelativeYaw = shotVector.getAngle().getRadians();
    double requiredHorizontalVelocity = shotVector.getNorm();
    double requiredPitch = Math.acos((requiredHorizontalVelocity / projectileVelocity));

    return new TargetingResult3d(
        requiredPitch, fieldRelativeYaw, distance / requiredHorizontalVelocity);
  }
}
