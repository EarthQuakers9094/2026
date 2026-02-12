package frc.robot.util;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class FieldUtil {
  public static boolean isNearTrench(Pose2d pose) {
    Pose2d normalizedPose =
        normalizePose2d(pose, DriverStation.getAlliance().orElse(Alliance.Blue));

    Translation2d translation = normalizedPose.getTranslation();

    if (translation.getX() < Constants.Field.trenchOrigin.in(Meters)
        || translation.getX()
            > Constants.Field.trenchOrigin.plus(Constants.Field.trenchWidth).in(Meters)) {
      Logger.recordOutput("IsNearTrench", false);
      return false;
    }

    if (translation.getY() > Constants.Field.trenchHeight.in(Meters)) {
      Logger.recordOutput("IsNearTrench", false);
      return false;
    }

    Logger.recordOutput("IsNearTrench", true);
    return true;
  }

  public static Translation2d normalizeTranslation2d(Translation2d translation, Alliance alliance) {
    double x = translation.getX();
    double y = translation.getY();
    if (alliance.equals(Alliance.Red)) {
      x = Constants.Field.fieldLength - x;
      y = Constants.Field.fieldWidth - y;
    }
    if (y > Constants.Field.fieldWidth / 2) {
      y = Constants.Field.fieldWidth - y;
    }
    return new Translation2d(x, y);
  }

  public static Rotation2d normalizeRotation2d(Rotation2d rotation, Alliance alliance) {
    Rotation2d theta = rotation;
    if (alliance.equals(Alliance.Red)) {
      theta = theta.plus(Rotation2d.k180deg);
    }
    return theta;
  }

  public static Pose2d normalizePose2d(Pose2d pose, Alliance alliance) {
    Translation2d translation = normalizeTranslation2d(pose.getTranslation(), alliance);
    Rotation2d rotation = pose.getRotation();
    if (translation.getY() > Constants.Field.fieldWidth / 2) {
      rotation = rotation.plus(Rotation2d.kCCW_90deg);
    }
    return new Pose2d(translation, normalizeRotation2d(pose.getRotation(), alliance));
  }
}
