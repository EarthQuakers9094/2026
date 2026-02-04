package frc.robot.commands;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.shooter.targeter.Targeter;
import frc.robot.subsystems.shooter.targeter.Targeter.TargetingData;
import frc.robot.subsystems.shooter.targeter.TargetingResult.TargetingResult3d;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class ShooterTrackTarget extends Command {

  private final ShooterSubsystem shooterSubsystem;
  private final Targeter targeter;
  private final Supplier<Pose2d> robotPositionSupplier;
  private final Supplier<ChassisSpeeds> chassisSpeedsSupplier;
  private final Translation3d target;

  public ShooterTrackTarget(
      ShooterSubsystem shooterSubsystem,
      Supplier<Pose2d> robotPositionSupplier,
      Supplier<ChassisSpeeds> chassisSpeedsSupplier,
      Targeter targeter,
      Translation3d target) {
    this.shooterSubsystem = shooterSubsystem;
    this.targeter = targeter;
    this.robotPositionSupplier = robotPositionSupplier;
    this.chassisSpeedsSupplier = chassisSpeedsSupplier;
    this.target = target;

    addRequirements(shooterSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    Pose2d robotPosition = robotPositionSupplier.get();
    ChassisSpeeds chassisSpeeds = chassisSpeedsSupplier.get();
    Pose2d anticipatedRobotPosition =
        robotPosition.exp(
            chassisSpeeds.toTwist2d(Constants.ShooterConstants.robotPositionAnticipationSeconds));
    Logger.recordOutput("Shooter/AnticipatedRobotPosition", anticipatedRobotPosition);

    Pose2d anticipatedShooterPosition =
        anticipatedRobotPosition.transformBy(
            new Transform2d(
                Constants.ShooterConstants.positionOnRobot.getTranslation().toTranslation2d(),
                Constants.ShooterConstants.positionOnRobot.getRotation().toRotation2d()));

    Distance xDistance =
        Meters.of(
            target.toTranslation2d().getDistance(anticipatedShooterPosition.getTranslation()));
    Distance yDistance =
        Meters.of(target.getZ() - Constants.ShooterConstants.positionOnRobot.getZ());
    double robotVelocityMps =
        Math.sqrt(
            Math.pow(chassisSpeeds.vyMetersPerSecond, 2)
                + Math.pow(chassisSpeeds.vxMetersPerSecond, 2));
    double shooterToTargetAngle =
        (target.toTranslation2d().minus(anticipatedShooterPosition.getTranslation()))
            .getAngle()
            .getRadians();
    double velocityAngleRelativeToTarget =
        Math.atan2(chassisSpeeds.vyMetersPerSecond, chassisSpeeds.vxMetersPerSecond)
            - shooterToTargetAngle;
    LinearVelocity xVelocity =
        MetersPerSecond.of(Math.cos(velocityAngleRelativeToTarget) * robotVelocityMps);
    LinearVelocity zVelocity =
        MetersPerSecond.of(Math.sin(velocityAngleRelativeToTarget) * robotVelocityMps);
    LinearVelocity projectileVelocity =
        MetersPerSecond.of(
            Constants.ShooterConstants.flywheelDiameter.in(Meters)
                * shooterSubsystem.getShooterSpeed().in(RadiansPerSecond));

    TargetingResult3d targetingResult =
        targeter.getShooterTargeting(
            new TargetingData(xDistance, yDistance, xVelocity, zVelocity, projectileVelocity));

    shooterSubsystem.setYaw(
        new Rotation2d(shooterToTargetAngle + targetingResult.yawRadians())
            .minus(robotPosition.getRotation()));
    if (targetingResult.pitchRadians() >= Math.PI / 2.) {
      DriverStation.reportError(
          "Targeting pitch result ("
              + targetingResult.pitchRadians()
              + ") is outside of physically possible range.",
          true);
      // TODO: Not sure what to do in this case... we're asking just too much of our shooter
    } else {
      shooterSubsystem.setPitch(new Rotation2d(targetingResult.pitchRadians()));
    }
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
