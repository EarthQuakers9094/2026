package frc.robot.subsystems.shooter.targeter;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.subsystems.shooter.targeter.TargetingResult.TargetingResult2d;
import frc.robot.subsystems.shooter.targeter.TargetingResult.TargetingResult3d;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;

public class IterativeKinematicTargeter implements KinematicTargeter {

  private final int iterations;

  private double iterationStartTime;
  private double targetingStartTime;

  public IterativeKinematicTargeter(int iterations) {
    this.iterations = iterations;
    if (iterations <= 0) {
      throw new IllegalArgumentException("iterations must be greater than zero");
    }
  }

  @Override
  public Optional<TargetingResult3d> getShooterTargeting(TargetingData targetingData) {
    double xMeters = targetingData.target().getNorm();
    double yMeters =
        targetingData.targetHeight().in(Meters) - Constants.ShooterConstants.positionOnRobot.getZ();
    double vMps = targetingData.projectileVelocity().in(MetersPerSecond);
    Translation2d directionToTarget = targetingData.robotVelocity().div(xMeters);
    Translation2d robotVelocity = targetingData.robotVelocity();
    double velocityTowardsTargetMps =
        directionToTarget.dot(robotVelocity) * robotVelocity.getNorm();
    double velocityPerpendicularToTargetMps =
        Math.sqrt(robotVelocity.getSquaredNorm() - Math.pow(velocityTowardsTargetMps, 2));

    double xMetersLookahead = xMeters;

    TargetingResult2d targetingResult = null;

    targetingStartTime = Timer.getFPGATimestamp();
    for (int i = 0; i < iterations; i++) {
      iterationStartTime = Timer.getFPGATimestamp();
      targetingResult = getShooterTargetingWithoutVelocity(xMetersLookahead, yMeters, vMps);
      xMetersLookahead = xMeters - targetingResult.timeOfFlightSeconds() * velocityTowardsTargetMps;
      Logger.recordOutput("Shooter/Targeter/xMetersLookahead", xMetersLookahead);
      Logger.recordOutput("Shooter/Targeter/velocityTowardsTarget", velocityTowardsTargetMps);
      Logger.recordOutput("Shooter/Targeter/xMeters", xMeters);
      Logger.recordOutput("Shooter/Targeter/timeOfFlight", targetingResult.timeOfFlightSeconds());

      Logger.recordOutput(
          "Shooter/IterationDurationMs", 1000 * (Timer.getFPGATimestamp() - iterationStartTime));
    }
    Logger.recordOutput(
        "Shooter/TargetingDurationMs", 1000 * (Timer.getFPGATimestamp() - targetingStartTime));

    // Targeting result cannot be null at this point since iterations must be >= 1
    Logger.recordOutput(
        "Shooter/Targeter/zMetersLookahead",
        velocityPerpendicularToTargetMps * targetingResult.timeOfFlightSeconds());

    return Optional.of(
        new TargetingResult3d(
            targetingResult.pitchRadians(),
            directionToTarget.getAngle().getRadians()
                - Math.atan2(
                    velocityPerpendicularToTargetMps * targetingResult.timeOfFlightSeconds(),
                    xMeters - velocityTowardsTargetMps * targetingResult.timeOfFlightSeconds()),
            targetingResult.timeOfFlightSeconds()));
  }
}
