package frc.robot.subsystems.shooter.targeter;

public class TargetingResult {
  public record TargetingResult2d(double pitchRadians, double timeOfFlightSeconds) {}

  public record TargetingResult3d(
      double pitchRadians, double yawRadians, double timeOfFlightSeconds) {}
}
