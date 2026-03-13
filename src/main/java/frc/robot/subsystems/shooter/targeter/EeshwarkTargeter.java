package frc.robot.subsystems.shooter.targeter;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.shooter.targeter.TargetingResult.TargetingResult3d;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;

public class EeshwarkTargeter implements Targeter {

  private InterpolatingDoubleTreeMap distanceToRPM = new InterpolatingDoubleTreeMap();
  // private InterpolatingDoubleTreeMap distanceToTOF = new
  // InterpolatingDoubleTreeMap();

  private InterpolatingDoubleTreeMap velocityToDistance = new InterpolatingDoubleTreeMap();

  public EeshwarkTargeter() {
    distanceToRPM.put(4.776835, 3600d);
    distanceToRPM.put(4.305231, 3400d);
    distanceToRPM.put(4.09, 3300d);

    distanceToRPM.put(3.943729, 3250d);

    distanceToRPM.put(3.63685, 3100d);

    distanceToRPM.put(2.878153, 2900d);

    distanceToRPM.put(2.652807, 2800d);

    distanceToRPM.put(2.200375, 2700d);

    distanceToRPM.put(1.918899, 2600d);

    double minDistance = 1.918899;
    double maxDistance = 4.776835;

    // distanceToTOF.put(null, null);
    // distanceToTOF.put(null, null);
    // distanceToTOF.put(null, null);
    // distanceToTOF.put(null, null);
    // distanceToTOF.put(null, null);
    // distanceToTOF.put(null, null);
    // distanceToTOF.put(null, null);

    for (int i = 0; i < 10; i++) {
      // double distance = ((maxTOFDistance - minTOFDistance) / 10) * i +
      double distance = (((maxDistance - minDistance) / 10) * i) + minDistance;

      double launchAngle = ShooterSubsystem.getIdealPitch(distance);

      double velocity =
          ShooterSubsystem.shooterSpeedToVelocity(distanceToRPM.get(distance) * (Math.PI / 30.));
      double xVelocity = Math.cos(launchAngle) * velocity;

      // minTOFDistance;
      velocityToDistance.put(xVelocity, distance);
    }
  }

  // @AutoLogOutput
  // public static AngularVelocity getIdealShooterSpeed(double distanceToTarget) {
  //   Logger.recordOutput("DistanceToTargetMeters", distanceToTarget);
  //   double rpm = 175.67282 * distanceToTarget + 2615.69268; // SmartDashboard.getNumber("RPM",
  // 0.0);
  //   return RPM.of(rpm);
  //   // if (distanceToTarget > 2.0) {
  //   // Logger.recordOutput("Shooter/DistanceToTarget", "far");
  //   // return RPM.of(3500);
  //   // } else {
  //   // Logger.recordOutput("Shooter/DistanceToTarget", "near");
  //   // return RPM.of(3000);
  //   // }
  // }
  private static double getIdealPitch(double distanceToTarget) {
    return -0.180371 * distanceToTarget + 1.6617; // -0.128837 * distanceToTarget + 1.58586;
  }

  public double calculateAdjustedRpm(double requiredVelocity) {
    double effectiveDistance = velocityToDistance.get(requiredVelocity);
    Logger.recordOutput("EffectiveDistance", effectiveDistance);
    return distanceToRPM.get(effectiveDistance);
  }

  @Override
  public Optional<TargetingResult3d> getShooterTargeting(TargetingData targetingData) {

    // double projectileVelocity = targetingData.projectileVelocity().in(MetersPerSecond);
    // double projectileVelocity =
    // targetingData.projectileVelocity().in(MetersPerSecond);
    double distance = targetingData.target().getNorm();
    Logger.recordOutput("DistancePassedToTargeter", distance);
    // Logger.recordOutput("TOF", distanceToTOF.get(distance));

    double idealPitch = getIdealPitch(distance);

    Translation2d directionToTarget = targetingData.target().div(distance);
    // TargetingResult2d staticVelocity =
    // this.getShooterTargetingWithoutVelocity(
    // distance,
    // targetingData.targetHeight().in(Meters)
    // - Constants.ShooterConstants.positionOnRobot.getZ(),
    // projectileVelocity);
    double baseRPM = distanceToRPM.get(distance);
    double projectileVelocity = ShooterSubsystem.shooterSpeedToVelocity(baseRPM * (Math.PI / 30.));
    double staticHorizontalVelocity = Math.cos(idealPitch) * projectileVelocity;

    Translation2d staticShotVelocity = directionToTarget.times(staticHorizontalVelocity);

    Translation2d shotVector = staticShotVelocity.plus(targetingData.robotVelocity());

    if (shotVector.getSquaredNorm() == 0
        || Double.isNaN(shotVector.getX())
        || Double.isNaN(shotVector.getY())) {
      return Optional.empty();
    }

    Rotation2d shotYaw = shotVector.getAngle();
    double fieldRelativeYaw = shotYaw.getRadians();
    double requiredHorizontalVelocity = shotVector.getNorm();
    // double requiredVelocity = requiredHorizontalVelocity /
    // targetingData.launchAngle().getCos();

    // double requiredPitch = Math.acos((requiredHorizontalVelocity / projectileVelocity));

    return Optional.of(
        new TargetingResult3d(
            idealPitch,
            calculateAdjustedRpm(requiredHorizontalVelocity),
            fieldRelativeYaw,
            distance / requiredHorizontalVelocity));
  }
}
