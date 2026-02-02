package frc.robot.subsystems.shooter.targeter;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.shooter.targeter.TargetingResult.TargetingResult2d;
import frc.robot.subsystems.shooter.targeter.TargetingResult.TargetingResult3d;
import org.littletonrobotics.junction.Logger;

public class IterativeKinematicTargeter implements Targeter {

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
  public TargetingResult3d getShooterTargeting(TargetingData targetingData) {
    double xMeters = targetingData.horizontalDistanceToTarget().in(Meters);
    double yMeters = targetingData.verticalDistanceToTarget().in(Meters);
    double vMps = targetingData.projectileVelocity().in(MetersPerSecond);
    double velocityTowardsTargetMps = targetingData.velocityTowardsTarget().in(MetersPerSecond);
    double velocityPerpendicularToTargetMps =
        targetingData.velocityPerpendicularToTarget().in(MetersPerSecond);

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

    return new TargetingResult3d(
        targetingResult.pitchRadians(),
        -Math.atan2(
            velocityPerpendicularToTargetMps * targetingResult.timeOfFlightSeconds(),
            xMeters - velocityTowardsTargetMps * targetingResult.timeOfFlightSeconds()),
        targetingResult.timeOfFlightSeconds());
  }

  private TargetingResult2d getShooterTargetingWithoutVelocity(
      double xMeters, double yMeters, double vMps) {
    double v_squared = Math.pow(vMps, 2);
    double g = 9.81;

    double pitchRadians =
        Math.atan2(
            v_squared
                + Math.sqrt(
                    Math.pow(v_squared, 2)
                        - g * (g * Math.pow(xMeters, 2) + 2 * yMeters * v_squared)),
            g * xMeters);

    double timeOfFlightSeconds = xMeters / (vMps * Math.cos(pitchRadians));

    return new TargetingResult2d(pitchRadians, timeOfFlightSeconds);
  }
}
