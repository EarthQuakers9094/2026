package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.MovingAverage;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class ShooterSubsystem extends SubsystemBase {

  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();
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
      BooleanSupplier shouldTargetHub) {
    this.io = io;
    this.robotPositionSupplier = robotPositionSupplier;
    this.chassisSpeedsSupplier = chassisSpeedsSupplier;
    this.shouldTargetHub = shouldTargetHub;
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

  @Override
  public void periodic() {
    io.updateInputs(inputs);

    if (shouldTargetHub.getAsBoolean()) {
      Pose2d robotPosition = robotPositionSupplier.get();
      Translation3d hubTarget = Constants.Field.hubTarget;
      Pose2d shooter =
          robotPosition.plus(
              new Transform2d(
                  Constants.ShooterConstants.positionOnRobot.toTranslation2d(), inputs.currentYaw));

      Translation2d shooterToTarget = hubTarget.toTranslation2d().minus(shooter.getTranslation());

      double x =
          Math.sqrt(
              Math.pow(shooterToTarget.getX(), 2)
                  + Math.pow(shooterToTarget.getY(), 2)); // target distance in meters
      double y = hubTarget.getZ() - Constants.ShooterConstants.positionOnRobot.getZ(); // target
      // height in
      // meters
      //
      ChassisSpeeds chassisSpeeds = chassisSpeedsSupplier.get();
      double driveVelocity =
          Math.sqrt(
              Math.pow(chassisSpeeds.vxMetersPerSecond, 2)
                  + Math.pow(chassisSpeeds.vyMetersPerSecond, 2));
      double driveAngle =
          Math.atan2(chassisSpeeds.vyMetersPerSecond, chassisSpeeds.vxMetersPerSecond);

      double driveAngleRelativeToTarget = driveAngle - shooterToTarget.getAngle().getRadians();

      double velocityTowardsTarget = Math.cos(driveAngleRelativeToTarget) * driveVelocity;
      double velocityPerpendicularToTarget = Math.sin(driveAngleRelativeToTarget) * driveVelocity;

      Logger.recordOutput("Shooter/DriveAngleRelativeToTarget", driveAngleRelativeToTarget);
      Logger.recordOutput("Shooter/VelocityTowardsTarget", velocityTowardsTarget);
      Logger.recordOutput(
          "Shooter/RobotVelocityMagnitude", driveVelocity - Math.abs(velocityTowardsTarget));

      TargetingResult3d targetingResult =
          approximateTargetingWithVelocity(
              x, y, velocityTowardsTarget, velocityPerpendicularToTarget);

      // https://en.wikipedia.org/wiki/Projectile_motion#Angle_%CE%B8_required_to_hit_coordinate_(x,_y)

      Logger.recordOutput("Shooter/TargetingHeight", y);
      // Logger.recordOutput("Shooter/TargetingVelocity", v);
      Logger.recordOutput("Shooter/TargetingDistance", x);
      Logger.recordOutput("Shooter/TargetingPitch", targetingResult.pitchRadians);

      io.setPitch(new Rotation2d(targetingResult.pitchRadians));
      io.setYaw(
          shooterToTarget
              .getAngle()
              .minus(robotPosition.getRotation())
              .plus(new Rotation2d(targetingResult.yawRadians /* + Math.PI*/)));
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

  private TargetingResult3d approximateTargetingWithVelocity(
      double xDistanceMeters,
      double yDistanceMeters,
      double xVelocityMetersPerSecond,
      double zVelocityMetersPerSecond) {
    double x = xDistanceMeters;
    TargetingResult2d newestTargetingResult2d = null;
    for (int i = 0; i < Constants.ShooterConstants.targetingIterations; i++) {
      newestTargetingResult2d = getTargetingForDistance(x, yDistanceMeters);
      x = xDistanceMeters - xVelocityMetersPerSecond * newestTargetingResult2d.timeOfFlightSeconds;
    }
    return new TargetingResult3d(
        newestTargetingResult2d.pitchRadians,
        -Math.atan2(
            zVelocityMetersPerSecond * newestTargetingResult2d.timeOfFlightSeconds,
            xDistanceMeters),
        newestTargetingResult2d.timeOfFlightSeconds);
  }

  private TargetingResult2d getTargetingForDistance(
      double xDistanceMeters, double yDistanceMeters) {
    double v =
        inputs.shooterSpeed.in(RadiansPerSecond)
            * Constants.ShooterConstants.flywheelDiameter.in(Meters);
    double v_squared = Math.pow(v, 2);
    double g = 9.81;

    double pitchRadians =
        Math.atan2(
            v_squared
                + Math.sqrt(
                    Math.pow(v_squared, 2)
                        - g * (g * Math.pow(xDistanceMeters, 2) + 2 * yDistanceMeters * v_squared)),
            g * xDistanceMeters);

    // double pitchRadians =
    //     Math.atan2(
    //         v_squared
    //             + Math.sqrt(
    //                 Math.pow(v_squared, 2)
    //                     - g * (g * Math.pow(xDistanceMeters, 2) + 2 * yDistanceMeters *
    // v_squared)),
    //         g * xDistanceMeters);
    double timeOfFlightSeconds = xDistanceMeters / (v * Math.cos(pitchRadians));

    return new TargetingResult2d(pitchRadians, timeOfFlightSeconds);
  }

  private record TargetingResult2d(double pitchRadians, double timeOfFlightSeconds) {}

  private record TargetingResult3d(
      double pitchRadians, double yawRadians, double timeOfFlightSeconds) {}
}
