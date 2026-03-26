package frc.robot.subsystems.shooter.targeter;

import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants;
import frc.robot.subsystems.shooter.targeter.TargetingResult.TargetingResult3d;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class MechanicalAdvantageTargeter implements Targeter {

  private LoggedNetworkBoolean twistCompensation =
      new LoggedNetworkBoolean("TwistCompensation", true);

  private LoggedNetworkNumber twistCompensationFactor =
      new LoggedNetworkNumber("TwistCompensationFactor", 1.0);

  @Override
  public Optional<TargetingResult3d> getShooterTargeting(TargetingData targetingData) {
    double distance = targetingData.target().getNorm();
    Logger.recordOutput("DistancePassedToTargeter", distance);

    Translation2d robotVelocity = targetingData.robotVelocity();

    // if (twistCompensation.get()) {
      Translation2d robotToShooter =
          Constants.ShooterConstants.positionOnRobot.getTranslation().toTranslation2d();
      double twistRadius = robotToShooter.getNorm();
      double tangentialVelocity =
          twistCompensationFactor.get()
              * targetingData.robotOmegaAngularVelocity().in(RadiansPerSecond)
              * twistRadius;

      // If something is going majorly screwy with our targeting when this switch is
      // on, it is almost certainly this godless affront to math in the following ~6
      // lines

      double velocityAngleRelativeToFieldAxis =
          targetingData.robotRotation().getRadians() - robotToShooter.getAngle().getRadians();
      Translation2d rotationalVelocity =
          new Translation2d(
              Math.cos(velocityAngleRelativeToFieldAxis) * tangentialVelocity,
              Math.sin(velocityAngleRelativeToFieldAxis) * tangentialVelocity);

      robotVelocity = robotVelocity.plus(rotationalVelocity);
    // }

    Translation2d lookaheadTarget = targetingData.target();

    for (int i = 0; i <= 40; i++) {
      ShotParams params = EeshwarkTargeter.shotMap.get(lookaheadTarget.getNorm());

      lookaheadTarget = targetingData.target().minus(robotVelocity.times(params.TOF()).plus(targetingData.robotAcceleration().times(0.5 * Math.pow(params.TOF(),2))));
      // lookaheadTarget.rotateBy(
      //     new Rotation2d(
      //         targetingData.robotOmegaAngularVelocity().in(RadiansPerSecond)
      //             * params.TOF()
      //             * twistCompensationFactor.get()));
    }

    Translation2d shotDirection = lookaheadTarget.div(lookaheadTarget.getNorm());

    if (shotDirection.getSquaredNorm() == 0
        || Double.isNaN(shotDirection.getX())
        || Double.isNaN(shotDirection.getY())) {
      return Optional.empty();
    }

    Rotation2d shotYaw = shotDirection.getAngle();
    double fieldRelativeYaw = shotYaw.getRadians();
    ShotParams params = EeshwarkTargeter.shotMap.get(lookaheadTarget.getNorm());

    return Optional.of(
        new TargetingResult3d(params.hoodPosition(), params.RPM(), fieldRelativeYaw, params.TOF()));
  }
}
