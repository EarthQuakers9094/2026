package frc.robot.commands;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.shooter.targeter.Targeter;
import java.util.Optional;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class DriverAutomations {
  public static Command targetHubOrFerry(
      ShooterSubsystem shooterSubsystem,
      Supplier<Pose2d> robotPoseSupplier,
      Supplier<ChassisSpeeds> chassisSpeedsSupplier,
      Targeter targeter) {

    return new ShooterTrackTarget(
        shooterSubsystem,
        robotPoseSupplier,
        chassisSpeedsSupplier,
        targeter,
        () -> selectTarget(robotPoseSupplier.get(), DriverStation.getAlliance()),
        true);
  }

  private static Translation3d selectTarget(Pose2d pose, Optional<Alliance> maybeAlliance) {
    Alliance alliance = maybeAlliance.orElse(Alliance.Blue);
    if (inAllianceZone(pose, alliance)) {
      return Constants.Field.hubTarget;
    }

    double y = pose.getMeasureY().in(Meters);
    if (alliance.equals(Alliance.Red)) {
      y = Constants.Field.fieldWidth - y;
    }
    double targetY = Inches.of(200).in(Meters);
    if (y < Constants.Field.fieldWidth / 2) {
      targetY = Constants.Field.fieldWidth - targetY;
    }
    return new Translation3d(Inches.of(120.0), Meters.of(targetY), Meters.zero());
  }

  //   @AutoLogOutput
  public static boolean inAllianceZone(Pose2d pose, Alliance alliance) {
    double x = pose.getMeasureX().in(Meters);
    if (alliance.equals(Alliance.Red)) {
      x = Constants.Field.fieldLength - x;
    }

    // Logger.recordOutput("Pose", null);

    Logger.recordOutput("IsInAllianceZone", x < Constants.Field.allianceZoneWidth.in(Meters));
    return x < Constants.Field.allianceZoneWidth.in(Meters);
  }
}
