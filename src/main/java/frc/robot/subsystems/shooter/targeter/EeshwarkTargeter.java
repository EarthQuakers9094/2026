package frc.robot.subsystems.shooter.targeter;

import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import frc.robot.Constants;
import frc.robot.subsystems.shooter.targeter.TargetingResult.TargetingResult3d;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

public class EeshwarkTargeter implements Targeter {

  private LoggedNetworkBoolean twistCompensation =
      new LoggedNetworkBoolean("TwistCompensation", false);

  public static InterpolatingTreeMap<Double, ShotParams> shotMap =
      new InterpolatingTreeMap<Double, ShotParams>(
          MathUtil::inverseInterpolate, Targeter::shotInterpolator);
  // private InterpolatingDoubleTreeMap distanceToTOF = new
  // InterpolatingDoubleTreeMap();

  private static InterpolatingDoubleTreeMap velocityToDistance = new InterpolatingDoubleTreeMap();
  private LoggedNetworkBoolean correctWithRPM = new LoggedNetworkBoolean("CorrectWithRPM", true);
  private double TOF = 0.0;

  static {

    // dcmp data
    shotMap.put(6.3069, new ShotParams(3750, 2.1, 47.90 - 46.60));

    shotMap.put(5.73245, new ShotParams(3650, 1.95, 52.20 - 51.0));

    shotMap.put(5.001142, new ShotParams(3400, 1.702637, 16.35 - 15.09));
    shotMap.put(4.4543455, new ShotParams(3250, 1.4, 53.04 - 51.81));
    shotMap.put(4.139972, new ShotParams(3200, 1.35, 52.61 - 51.31));
    shotMap.put(3.756959, new ShotParams(3125, 1.25, 6.85 - 5.5));
    shotMap.put(3.36106, new ShotParams(3025, 1.125, 5.80 - 4.54));
    shotMap.put(2.934494, new ShotParams(2875, 0.9, 10.3 - 8.99));
    shotMap.put(2.3173, new ShotParams(2760, 0.7, 9.82 - 8.67));
    shotMap.put(1.8439404, new ShotParams(2760, 0.5, 23.64 - 22.59));
    shotMap.put(1.4798, new ShotParams(2760, 0.4, 15.58 - 14.32));
    // shotMap.put(, new ShotParams(, ,   - ));
    // shotMap.put(, new ShotParams(, ,   - ));
    // shotMap.put(, new ShotParams(, ,   - ));
    // shotMap.put(, new ShotParams(, ,   - ));

    // olddata
    // shotMap.put(3.0463328824003626, new ShotParams(3000, 1.6, 2.7 - 1.58));
    // // shotMap.put(2.8789923633033014, new ShotParams(2800, 1.61, 2.34-1.35)); BAD
    // // DATA POINT
    // shotMap.put(4.237791791973659, new ShotParams(3300, 2.2, 1.56 - 0.32));
    // shotMap.put(3.8188085361104056, new ShotParams(3200, 2.1, 1.94 - 0.84));
    // shotMap.put(3.4200357766207268, new ShotParams(3050, 2.025, 3.08 - 1.91));
    // shotMap.put(2.7163413872695594, new ShotParams(2850, 1.4, 1.42 - 0.38));
    // shotMap.put(2.4258430558322117, new ShotParams(2800, 1.3, 1.44 - 0.32));
    // shotMap.put(2.119235489902627, new ShotParams(2650, 1.05, 1.74 - 0.75));
    // shotMap.put(1.904048127971999, new ShotParams(2600, 0.925, 1.68 - 0.73));
    // shotMap.put(1.5355979615043405, new ShotParams(2600, 0.6, 1.42 - 0.36));
    // shotMap.put(1.0561370725815047, new ShotParams(2500, 0.35, 3.33 - 2.29));
    // shotMap.put(4.964619639141648, new ShotParams(3600, 2.35, 3.39 - 1.96));
    // shotMap.put(4.964619639141648, new ShotParams(3600, 2.35, 3.39 - 1.96));    //
    // shotMap.put(4.964619639141648, new ShotParams(3600, 2.35, 3.39 - 1.96));

    // double minDistance = 1.0561370725815047;
    // double maxDistance = 4.964619639141648;

    // Newer data before dcmp
    // shotMap.put(5.042064448325879, new ShotParams(3385, 2.246, 4.56 - 3.27));
    // shotMap.put(4.692851868222061, new ShotParams(3350, 2.1, 5.84 - 4.64));
    // shotMap.put(4.263325456381816, new ShotParams(3250, 1.9, 4.39 - 3.22));
    // shotMap.put(3.9220137708683636, new ShotParams(3050, 1.8, 3.63 - 2.5));
    // shotMap.put(3.6095082462299666, new ShotParams(3025, 1.55, 3.62 - 2.44));
    // shotMap.put(3.208731477252873, new ShotParams(2915, 1.4, 3.62 - 2.52));
    // shotMap.put(2.8736308316568264, new ShotParams(2880, 1.2, 3.59 - 2.56));
    // shotMap.put(2.6087508103794472, new ShotParams(2730, 1.15, 3.54 - 2.52));
    // shotMap.put(2.284140519891348, new ShotParams(2650, 0.9, 19.76 - 18.62));
    // shotMap.put(2.105161339906001, new ShotParams(2625, 0.85, 5.17 - 4.11));
    // shotMap.put(1.8371400611905015, new ShotParams(2600, 0.8, 3.49 - 2.52));
    // shotMap.put(1.2942615663372328, new ShotParams(2575, 0.1, 15.08 - 13.91));
    // distanceToTOF.put(null, null);
    // distanceToTOF.put(null, null);
    // distanceToTOF.put(null, null);
    // distanceToTOF.put(null, null);
    // distanceToTOF.put(null, null);
    // distanceToTOF.put(null, null);
    // distanceToTOF.put(null, null);

    // for (int i = 0; i < 15; i++) {
    //   // double distance = ((maxTOFDistance - minTOFDistance) / 10) * i +
    //   double distance = (((maxDistance - minDistance) / 10) * i) + minDistance;
    //   double TOF = shotMap.get(distance).TOF();
    //   // double launchAngle = ShooterSubsystem.getIdealPitch(distance);

    //   // double velocity =
    //   // ShooterSubsystem.shooterSpeedToVelocity(distanceToRPM.get(distance) *
    //   // (Math.PI / 30.));
    //   // double xVelocity = Math.cos(launchAngle) * velocity;

    //   // minTOFDistance;

    //   // System.out.println("Velocity: " + (distance / TOF) + " & Distance: " +
    //   // distance);
    //   System.out.println(
    //       "Distance: " + distance + " & Velocity: " + (distance / TOF) + " & TOF: " + TOF);

    //   velocityToDistance.put(distance / TOF, distance);
    // }
  }

  // @AutoLogOutput
  // public static AngularVelocity getIdealShooterSpeed(double distanceToTarget) {
  // Logger.recordOutput("DistanceToTargetMeters", distanceToTarget);
  // double rpm = 175.67282 * distanceToTarget + 2615.69268; //
  // SmartDashboard.getNumber("RPM",
  // 0.0);
  // return RPM.of(rpm);
  // // if (distanceToTarget > 2.0) {
  // // Logger.recordOutput("Shooter/DistanceToTarget", "far");
  // // return RPM.of(3500);
  // // } else {
  // // Logger.recordOutput("Shooter/DistanceToTarget", "near");
  // // return RPM.of(3000);
  // // }
  // }
  private static double getIdealPitch(double distanceToTarget) {
    return -0.180371 * distanceToTarget + 1.6617; // -0.128837 * distanceToTarget + 1.58586;
  }

  public double calculateAdjustedRpm(double requiredVelocity) {
    Logger.recordOutput("RequiredVelocity", requiredVelocity);

    double effectiveDistance = velocityToDistance.get(requiredVelocity);
    Logger.recordOutput("EffectiveDistance", effectiveDistance);
    return shotMap.get(effectiveDistance).RPM();
  }

  public double calculateAdjustedHoodAngle(double requiredVelocity) {
    Logger.recordOutput("RequiredVelocity", requiredVelocity);

    double effectiveDistance = velocityToDistance.get(requiredVelocity);
    Logger.recordOutput("EffectiveDistance", effectiveDistance);
    return shotMap.get(effectiveDistance).hoodPosition();
  }

  @Override
  public Optional<TargetingResult3d> getShooterTargeting(TargetingData targetingData) {

    Translation2d robotVelocity = targetingData.robotVelocity();
    if (twistCompensation.get()) {
      Translation2d robotToShooter =
          Constants.ShooterConstants.positionOnRobot.getTranslation().toTranslation2d();
      double twistRadius = robotToShooter.getNorm();
      double tangentialVelocity =
          targetingData.robotOmegaAngularVelocity().in(RadiansPerSecond) * twistRadius;

      // If something is going majorly screwy with our targeting when this switch is
      // on, it is almost certainly this godless affront to math in the following ~6
      // lines

      double velocityAngleRelativeToFieldAxis =
          targetingData.robotRotation().getRadians() + robotToShooter.getAngle().getRadians();
      Translation2d rotationalVelocity =
          new Translation2d(
              Math.cos(velocityAngleRelativeToFieldAxis) * tangentialVelocity,
              Math.sin(velocityAngleRelativeToFieldAxis) * tangentialVelocity);

      robotVelocity = robotVelocity.plus(rotationalVelocity);
    }
    // double projectileVelocity =
    // targetingData.projectileVelocity().in(MetersPerSecond);
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
    TOF = params.TOF();
    // double projectileVelocity = ShooterSubsystem.shooterSpeedToVelocity(baseRPM *
    // (Math.PI /
    // 30.));
    double staticHorizontalVelocity = distance / params.TOF();

    Translation2d staticShotVelocity = directionToTarget.times(staticHorizontalVelocity);

    Translation2d shotVector = staticShotVelocity.minus(robotVelocity);

    if (shotVector.getSquaredNorm() == 0
        || Double.isNaN(shotVector.getX())
        || Double.isNaN(shotVector.getY())) {
      return Optional.empty();
    }

    Rotation2d shotYaw = shotVector.getAngle();
    double fieldRelativeYaw = shotYaw.getRadians();
    double requiredHorizontalVelocity = shotVector.getNorm();

    return Optional.of(
        new TargetingResult3d(
            calculateAdjustedHoodAngle(requiredHorizontalVelocity),
            calculateAdjustedRpm(requiredHorizontalVelocity),
            fieldRelativeYaw,
            distance / requiredHorizontalVelocity));
  }

  public double getTOF() {
    return TOF;
  }
}
