package frc.robot.subsystems.shooter.targeter;

import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import frc.robot.Constants;
import frc.robot.subsystems.shooter.targeter.TargetingResult.TargetingResult3d;
import java.util.Arrays;
import java.util.List;
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
  // private LoggedNetworkBoolean accelerationCompensation = new
  // LoggedNetworkBoolean("AccelerationCompensation",
  // false);
  private LoggedNetworkBoolean twistCompensation =
      new LoggedNetworkBoolean("TwistCompensation", false);

  public EeshwarkTargeter() {

    List<Pair<Double, ShotParams>> tableEntries =
        Arrays.asList(
            // new Pair<Double, ShotParams>(4.237791791973659, new ShotParams(3300, 2.2, 1.56 -
            // 0.32)), SUSPECT TOF
            // new Pair<Double, ShotParams>(
            //     4.773800137037311, new ShotParams(3625, 2.25, 12.28 - 10.71)),

            new Pair<Double, ShotParams>(
                4.964619639141648, new ShotParams(3600, 2.35, 3.39 - 1.96)),
            new Pair<Double, ShotParams>(4.17185280740973, new ShotParams(3300, 2.2, 4.06 - 2.83)),
            // new Pair<Double, ShotParams>(
            //     3.8188085361104056, new ShotParams(3200, 2.1, 1.94 - 0.84)),
            new Pair<Double, ShotParams>(
                3.4200357766207268, new ShotParams(3050, 2.025, 3.08 - 1.91)),
            new Pair<Double, ShotParams>(3.0463328824003626, new ShotParams(3000, 1.6, 2.7 - 1.58)),
            // new Pair<Double, ShotParams>(
            //     2.7163413872695594, new ShotParams(2850, 1.4, 1.42 - 0.38)),
            new Pair<Double, ShotParams>(
                2.4258430558322117, new ShotParams(2800, 1.3, 1.44 - 0.32)),
            new Pair<Double, ShotParams>(
                2.119235489902627, new ShotParams(2650, 1.05, 1.74 - 0.75)),
            new Pair<Double, ShotParams>(
                1.904048127971999, new ShotParams(2600, 0.925, 1.68 - 0.73)),
            // new Pair<Double, ShotParams>(
            //     1.5355979615043405, new ShotParams(2600, 0.6, 1.42 - 0.36)),
            new Pair<Double, ShotParams>(
                1.0561370725815047, new ShotParams(2500, 0.35, 3.33 - 2.29))

            // new Pair<Double, ShotParams>(5.817068469876448, new ShotParams(4000, 2.4, 2.59 -
            // 0.97)),
            // new Pair<Double, ShotParams>(
            //     6.684452643935784, new ShotParams(4500, 2.4, 4.18 - 2.33))
            );

    for (Pair<Double, ShotParams> pair : tableEntries) {
      shotMap.put(pair.getFirst(), pair.getSecond());
      // double distance = ((maxTOFDistance - minTOFDistance) / 10) * i +
      double distance = pair.getFirst();
      double TOF = pair.getSecond().TOF();
      // double launchAngle = ShooterSubsystem.getIdealPitch(distance);

      // double velocity =
      // ShooterSubsystem.shooterSpeedToVelocity(distanceToRPM.get(distance) *
      // (Math.PI / 30.));
      // double xVelocity = Math.cos(launchAngle) * velocity;

      // minTOFDistance;

      // System.out.println("Velocity: " + (distance / TOF) + " & Distance: " + distance);
      System.out.println(
          " & Distance: " + distance + "Velocity: " + (distance / TOF) + " & TOF: " + TOF);

      velocityToDistance.put(distance / TOF, distance);
    }
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

    double distance = targetingData.target().getNorm();
    Logger.recordOutput("DistancePassedToTargeter", distance);

    Translation2d directionToTarget = targetingData.target().div(distance);
    ShotParams params = shotMap.get(distance);

    double staticHorizontalVelocity = distance / params.TOF;

    Translation2d staticShotVelocity = directionToTarget.times(staticHorizontalVelocity);

    Translation2d shotVector = staticShotVelocity.minus(targetingData.robotVelocity());

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
}
