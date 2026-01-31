package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.shooter.targeter.Targeter;
import frc.robot.subsystems.shooter.targeter.Targeter.TargetingData;
import frc.robot.subsystems.shooter.targeter.TargetingResult.TargetingResult3d;
import frc.robot.util.MovingAverage;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class ShooterSubsystem extends SubsystemBase {

  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();
  private final Targeter targeter;
  private final Supplier<Pose2d> robotPositionSupplier;
  private final Supplier<ChassisSpeeds> chassisSpeedsSupplier;
  private final BooleanSupplier shouldTargetHub;
  private final MovingAverage speedAverage = new MovingAverage(40);

  @AutoLogOutput private boolean isShooting = false;
  @AutoLogOutput private boolean isIndexing = false;

  public ShooterSubsystem(
      ShooterIO io,
      Supplier<Pose2d> robotPositionSupplier,
      Supplier<ChassisSpeeds> chassisSpeedsSupplier,
      BooleanSupplier shouldTargetHub,
      Targeter targeter) {
    this.io = io;
    this.robotPositionSupplier = robotPositionSupplier;
    this.chassisSpeedsSupplier = chassisSpeedsSupplier;
    this.shouldTargetHub = shouldTargetHub;
    this.targeter = targeter;
  }

  public void beginShooting() {
    isShooting = true;
    io.setVelocitySetpoint(Constants.ShooterConstants.launchSpeed);
    Logger.recordOutput(
        "Shooter/SpeedSetpointRadPerSec",
        Constants.ShooterConstants.launchSpeed.in(RadiansPerSecond));
  }

  public void endShooting() {
    isShooting = false;
    io.setVelocitySetpoint(RPM.of(0.0));
    Logger.recordOutput("Shooter/SpeedSetpointRadPerSec", 0.0);
  }

  public boolean isActivelyShooting() {
    return this.isIndexing;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);

    if (shouldTargetHub.getAsBoolean()) {
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
              Constants.Field.hubTarget
                  .toTranslation2d()
                  .getDistance(anticipatedShooterPosition.getTranslation()));
      Distance yDistance =
          Meters.of(
              Constants.Field.hubTarget.getZ() - Constants.ShooterConstants.positionOnRobot.getZ());
      double robotVelocityMps =
          Math.sqrt(
              Math.pow(chassisSpeeds.vyMetersPerSecond, 2)
                  + Math.pow(chassisSpeeds.vxMetersPerSecond, 2));
      double shooterToTargetAngle =
          (Constants.Field.hubTarget
                  .toTranslation2d()
                  .minus(anticipatedShooterPosition.getTranslation()))
              .getAngle()
              .getRadians();
      double velocityAngleRelativeToTarget =
          Math.atan2(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond)
              - shooterToTargetAngle;
      LinearVelocity xVelocity =
          MetersPerSecond.of(Math.cos(velocityAngleRelativeToTarget) * robotVelocityMps);
      LinearVelocity zVelocity =
          MetersPerSecond.of(Math.sin(velocityAngleRelativeToTarget) * robotVelocityMps);
      LinearVelocity projectileVelocity =
          MetersPerSecond.of(
              Constants.ShooterConstants.flywheelDiameter.in(Meters)
                  * inputs.shooterSpeed.in(RadiansPerSecond));

      Logger.recordOutput(
          "Shooter/PerpendicularVelocity",
          new Pose2d[] {
            anticipatedShooterPosition,
            anticipatedRobotPosition.exp(
                new Twist2d(
                    zVelocity.baseUnitMagnitude() * Math.sin(shooterToTargetAngle),
                    zVelocity.baseUnitMagnitude() * Math.cos(shooterToTargetAngle),
                    0.0))
          });

      TargetingResult3d targetingResult =
          targeter.getShooterTargeting(
              new TargetingData(xDistance, yDistance, xVelocity, zVelocity, projectileVelocity));

      io.setPitch(new Rotation2d(targetingResult.pitchRadians()));
      io.setYaw(new Rotation2d(shooterToTargetAngle).minus(robotPosition.getRotation()));
    }
    double averageSpeed = speedAverage.addValue(inputs.shooterSpeed.baseUnitMagnitude());

    Logger.recordOutput("Shooter/AverageSpeed", averageSpeed);
    if (isShooting
        && averageSpeed >= Constants.ShooterConstants.minLaunchSpeed.baseUnitMagnitude()
        && speedAverage.getStandardDeviation(averageSpeed) <= 12.) {
      io.startIndexing();
      isIndexing = true;

    } else {
      io.stopIndexing();
      isIndexing = false;
    }
    Logger.processInputs("Shooter", inputs);
  }
}
