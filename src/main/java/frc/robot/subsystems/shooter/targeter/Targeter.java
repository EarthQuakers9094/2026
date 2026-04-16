package frc.robot.subsystems.shooter.targeter;

import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import frc.robot.subsystems.shooter.targeter.TargetingResult.TargetingResult3d;
import java.util.Optional;

public interface Targeter {
  public record TargetingData(
      Translation2d target,
      Distance targetHeight,
      Translation2d robotVelocity,
      Translation2d robotAcceleration,
      AngularVelocity robotOmegaAngularVelocity,
      Rotation2d robotRotation,
      boolean shouldFerry) {}

  public record RobotRelativeAcceleration(LinearAcceleration aX, LinearAcceleration aY) {

    public Translation2d toFieldRelative(Rotation2d rotation) {
      // Ugly hack that lets us not accidentally mess up the trig. I hate this btw.
      ChassisSpeeds fieldRelative =
          ChassisSpeeds.fromRobotRelativeSpeeds(
              this.aX.in(MetersPerSecondPerSecond),
              this.aY.in(MetersPerSecondPerSecond),
              0d,
              rotation);

      return new Translation2d(fieldRelative.vxMetersPerSecond, fieldRelative.vyMetersPerSecond);
    }

    public Twist2d toTwist2d(ChassisSpeeds initalVelocity, double dt) {
      // x = v*t + 1/2*a*t^2
      Twist2d baseTwist = initalVelocity.toTwist2d(dt);
      baseTwist.dx += 0.5 * this.aX.in(MetersPerSecondPerSecond) * Math.pow(dt, 2);
      baseTwist.dy += 0.5 * this.aY.in(MetersPerSecondPerSecond) * Math.pow(dt, 2);
      return baseTwist;
    }
  }

  public record ShotParams(double RPM, double hoodPosition, double TOF) {}

  public record FerryParams(double RPM, double TOF) {
    public ShotParams getShotParams() {
      return new ShotParams(this.RPM, 2.3, this.TOF);
    }
  }

  public Optional<TargetingResult3d> getShooterTargeting(TargetingData targetingData);

  public static ShotParams shotInterpolator(ShotParams start, ShotParams end, double t) {
    return new ShotParams(
        MathUtil.interpolate(start.RPM(), end.RPM(), t),
        MathUtil.interpolate(start.hoodPosition(), end.hoodPosition(), t),
        MathUtil.interpolate(start.TOF(), end.TOF(), t));
  }

  public static FerryParams ferryInterpolator(FerryParams start, FerryParams end, double t) {
    return new FerryParams(
        MathUtil.interpolate(start.RPM(), end.RPM(), t),
        MathUtil.interpolate(start.TOF(), end.TOF(), t));
  }

  public double getTOF();
}
