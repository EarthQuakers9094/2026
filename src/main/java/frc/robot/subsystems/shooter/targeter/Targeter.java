package frc.robot.subsystems.shooter.targeter;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.subsystems.shooter.targeter.TargetingResult.TargetingResult3d;

public interface Targeter {
  public record TargetingData(
      Distance horizontalDistanceToTarget,
      Distance verticalDistanceToTarget,
      LinearVelocity velocityTowardsTarget,
      LinearVelocity velocityPerpendicularToTarget,
      LinearVelocity projectileVelocity) {}

  public TargetingResult3d getShooterTargeting(TargetingData targetingData);
}
