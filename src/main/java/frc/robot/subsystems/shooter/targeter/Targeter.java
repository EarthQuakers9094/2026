package frc.robot.subsystems.shooter.targeter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import frc.robot.subsystems.shooter.targeter.TargetingResult.TargetingResult3d;
import java.util.Optional;

public interface Targeter {
  public record TargetingData(
      Translation2d target, Distance targetHeight, Translation2d robotVelocity) {}

  public record ShotParams(double RPM, double hoodPosition, double TOF) {}

  public Optional<TargetingResult3d> getShooterTargeting(TargetingData targetingData);

  public static ShotParams shotInterpolator(ShotParams start, ShotParams end, double t) {
    return new ShotParams(
        MathUtil.interpolate(start.RPM(), end.RPM(), t),
        MathUtil.interpolate(start.hoodPosition(), end.hoodPosition(), t),
        MathUtil.interpolate(start.TOF(), end.TOF(), t));
  }
}
