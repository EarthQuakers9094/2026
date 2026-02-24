package frc.robot.subsystems.shooter.targeter;

import frc.robot.subsystems.shooter.targeter.TargetingResult.TargetingResult2d;

public interface KinematicTargeter extends Targeter {
  public default TargetingResult2d getShooterTargetingWithoutVelocity(
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
