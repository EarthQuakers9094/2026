package frc.robot.subsystems.shooter.targeter;

import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;

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
      Rotation2d robotRotation) {}

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

  public Optional<TargetingResult3d> getShooterTargeting(TargetingData targetingData);
}
