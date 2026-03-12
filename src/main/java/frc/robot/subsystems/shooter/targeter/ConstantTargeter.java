package frc.robot.subsystems.shooter.targeter;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants;
import frc.robot.subsystems.shooter.targeter.TargetingResult.TargetingResult3d;
import java.util.Optional;

public class ConstantTargeter extends EeshwarkTargeter {

  @Override
  public Optional<TargetingResult3d> getShooterTargeting(TargetingData targetingData) {
    return super.getShooterTargeting(
        new TargetingData(
            Constants.Field.hubTarget.toTranslation2d(),
            targetingData.targetHeight(),
            new Translation2d()));
  }
}
