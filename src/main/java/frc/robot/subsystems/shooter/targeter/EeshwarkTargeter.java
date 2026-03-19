package frc.robot.subsystems.shooter.targeter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import frc.robot.subsystems.shooter.targeter.TargetingResult.TargetingResult3d;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

public class EeshwarkTargeter implements Targeter {

  public record ShotParams(double RPM, double hoodPosition, double TOF) {}

  public static ShotParams shotInterpolator(ShotParams start, ShotParams end, double t) {
    return new ShotParams(
        MathUtil.interpolate(start.RPM, end.RPM, t),
        MathUtil.interpolate(start.hoodPosition, end.hoodPosition, t),
        MathUtil.interpolate(start.TOF, end.TOF, t));
  }

  private InterpolatingTreeMap<Double, ShotParams> shotMap =
      new InterpolatingTreeMap<>(MathUtil::inverseInterpolate, EeshwarkTargeter::shotInterpolator);
  // private InterpolatingDoubleTreeMap distanceToTOF = new
  // InterpolatingDoubleTreeMap();

  private InterpolatingDoubleTreeMap velocityToDistance = new InterpolatingDoubleTreeMap();
  private LoggedNetworkBoolean correctWithRPM = new LoggedNetworkBoolean("CorrectWithRPM", true);

  public EeshwarkTargeter() {
    shotMap.put(3.0463328824003626, new ShotParams(3000, 1.6, 2.7 - 1.58));
    // shotMap.put(2.8789923633033014, new ShotParams(2800, 1.61, 2.34-1.35)); BAD DATA POINT
    shotMap.put(4.237791791973659, new ShotParams(3300, 2.2, 1.56 - 0.32));
    shotMap.put(3.8188085361104056, new ShotParams(3200, 2.1, 1.94 - 0.84));
    shotMap.put(3.4200357766207268, new ShotParams(3050, 2.025, 3.08 - 1.91));
    shotMap.put(2.7163413872695594, new ShotParams(2850, 1.4, 1.42 - 0.38));
    shotMap.put(2.4258430558322117, new ShotParams(2800, 1.3, 1.44 - 0.32));
    shotMap.put(2.119235489902627, new ShotParams(2650, 1.05, 1.74 - 0.75));
    shotMap.put(1.904048127971999, new ShotParams(2600, 0.925, 1.68 - 0.73));
    shotMap.put(1.5355979615043405, new ShotParams(2600, 0.6, 1.42 - 0.36));
    shotMap.put(1.0561370725815047, new ShotParams(2500, 0.35, 3.33 - 2.29));
    shotMap.put(4.964619639141648, new ShotParams(3600, 2.35, 3.39 - 1.96));

    double minDistance = 1.0561370725815047;
    double maxDistance = 4.964619639141648;

    // distanceToTOF.put(null, null);
    // distanceToTOF.put(null, null);
    // distanceToTOF.put(null, null);
    // distanceToTOF.put(null, null);
    // distanceToTOF.put(null, null);
    // distanceToTOF.put(null, null);
    // distanceToTOF.put(null, null);

    for (int i = 0; i < 15; i++) {
      // double distance = ((maxTOFDistance - minTOFDistance) / 10) * i +
      double distance = (((maxDistance - minDistance) / 10) * i) + minDistance;
      double TOF = shotMap.get(distance).TOF();
      // double launchAngle = ShooterSubsystem.getIdealPitch(distance);

      // double velocity =
      //     ShooterSubsystem.shooterSpeedToVelocity(distanceToRPM.get(distance) * (Math.PI / 30.));
      // double xVelocity = Math.cos(launchAngle) * velocity;

      // minTOFDistance;

      System.out.println("Velocity: " + (distance / TOF) + " & Distance: " + distance);
      velocityToDistance.put(distance / TOF, distance);
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
    Logger.recordOutput("RequiredVelocity", requiredVelocity);

    double effectiveDistance = velocityToDistance.get(requiredVelocity);
    Logger.recordOutput("EffectiveDistance", effectiveDistance);
    return shotMap.get(effectiveDistance).RPM;
  }

  public double calculateAdjustedHoodAngle(double requiredVelocity) {
    Logger.recordOutput("RequiredVelocity", requiredVelocity);

    double effectiveDistance = velocityToDistance.get(requiredVelocity);
    Logger.recordOutput("EffectiveDistance", effectiveDistance);
    return shotMap.get(effectiveDistance).hoodPosition;
  }

  @Override
  public Optional<TargetingResult3d> getShooterTargeting(TargetingData targetingData) {

    // double projectileVelocity = targetingData.projectileVelocity().in(MetersPerSecond);
    // double projectileVelocity =
    // targetingData.projectileVelocity().in(MetersPerSecond);
    double distance = targetingData.target().getNorm();
    Logger.recordOutput("DistancePassedToTargeter", distance);
    // Logger.recordOutput("TOF", distanceToTOF.get(distance));

    // double idealPitch = getIdealPitch(distance);

    Translation2d directionToTarget = targetingData.target().div(distance);
    // TargetingResult2d staticVelocity =
    // this.getShooterTargetingWithoutVelocity(
    // distance,
    // targetingData.targetHeight().in(Meters)
    // - Constants.ShooterConstants.positionOnRobot.getZ(),
    // projectileVelocity);
    ShotParams params = shotMap.get(distance);
    double baseRPM = params.RPM;
    // double projectileVelocity = ShooterSubsystem.shooterSpeedToVelocity(baseRPM * (Math.PI /
    // 30.));
    double staticHorizontalVelocity = distance / params.TOF;

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
            (correctWithRPM.getAsBoolean()
                ? params.hoodPosition
                : calculateAdjustedHoodAngle(requiredHorizontalVelocity)),
            (correctWithRPM.getAsBoolean()
                ? calculateAdjustedRpm(requiredHorizontalVelocity)
                : params.RPM),
            // calculateAdjustedRpm(requiredHorizontalVelocity),
            fieldRelativeYaw,
            distance / requiredHorizontalVelocity));
  }
}
