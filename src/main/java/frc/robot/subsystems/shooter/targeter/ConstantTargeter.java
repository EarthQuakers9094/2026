package frc.robot.subsystems.shooter.targeter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants;
import frc.robot.subsystems.shooter.targeter.TargetingResult.TargetingResult3d;
import frc.robot.util.AllianceFlipUtil;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;

public class ConstantTargeter extends EeshwarkTargeter {

  @Override
  public Optional<TargetingResult3d> getShooterTargeting(TargetingData targetingData) {
    Logger.recordOutput(
        "Constant target",
        AllianceFlipUtil.apply(
            new Pose2d(Constants.Field.hubTarget.toTranslation2d(), new Rotation2d())));
    return super.getShooterTargeting(
        new TargetingData(
            AllianceFlipUtil.apply(Constants.Field.hubTarget.toTranslation2d()),
            targetingData.targetHeight(),
            new Translation2d()));
  }
}
