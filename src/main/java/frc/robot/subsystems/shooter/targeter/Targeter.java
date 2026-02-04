package frc.robot.subsystems.shooter.targeter;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.subsystems.shooter.targeter.TargetingResult.TargetingResult3d;

public interface Targeter {
  public record TargetingData(
      Translation2d target,
      Distance targetHeight,
      Translation2d robotVelocity,
      LinearVelocity projectileVelocity) {}

  public TargetingResult3d getShooterTargeting(TargetingData targetingData);
}
