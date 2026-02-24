package frc.robot.subsystems.shooter.targeter;

import frc.robot.subsystems.shooter.targeter.TargetingResult.TargetingResult3d;
import java.util.Optional;

public class ConstantTargeter implements Targeter {

  @Override
  public Optional<TargetingResult3d> getShooterTargeting(TargetingData targetingData) {
    return Optional.of(new TargetingResult3d(Math.PI / 4., 0, 0));
  }
}
