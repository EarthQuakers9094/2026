package frc.robot.subsystems.shooter.targeter;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.shooter.targeter.TargetingResult.TargetingResult3d;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;

public class MechanicalAdvantageTargeter implements Targeter {

  @Override
  public Optional<TargetingResult3d> getShooterTargeting(TargetingData targetingData) {
    double distance = targetingData.target().getNorm();
    Logger.recordOutput("DistancePassedToTargeter", distance);

    Translation2d lookaheadTarget = targetingData.target();

    for (int i = 0; i <= 40; i++) {
      ShotParams params = EeshwarkTargeter.shotMap.get(lookaheadTarget.getNorm());

      lookaheadTarget =
          targetingData.target().minus(targetingData.robotVelocity().times(params.TOF()));
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
