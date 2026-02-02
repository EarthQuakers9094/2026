package frc.robot.subsystems.shooter.targeter;

import frc.robot.subsystems.shooter.targeter.TargetingResult.TargetingResult3d;

public class ConstantTargeter implements Targeter {

  @Override
  public TargetingResult3d getShooterTargeting(TargetingData targetingData) {
    return new TargetingResult3d(Math.PI / 4., 0, 0);
  }
}
