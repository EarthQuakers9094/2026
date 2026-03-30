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

  static {
    shotMap.put(5.098070710856465, new ShotParams(3660, 2.283, 4.96 - 3.390));
    shotMap.put(4.851891891966654, new ShotParams(3525, 2.283, 2.08 - 0.63));
    shotMap.put(4.558179560558487, new ShotParams(3430, 2.283, 3.63 - 2.23));
    shotMap.put(4.19749660974825, new ShotParams(3315, 2.283, 3.36 - 2.01));
    shotMap.put(3.9224328932130836, new ShotParams(3200, 2.283, 3.52 - 2.31));
    shotMap.put(3.7174510331104833, new ShotParams(3075, 2.283, 3.52 - 2.37));
    shotMap.put(3.519915355997788, new ShotParams(2975, 2.283, 3.35 - 2.15));
    shotMap.put(3.359358388831645, new ShotParams(2900, 2.283, 3.08 - 2.06));
    shotMap.put(3.099707376609207, new ShotParams(2830, 2.283, 3.14 - 2.12));
    shotMap.put(2.9017528375345853, new ShotParams(2700, 2.103, 2.98 - 2.06));
    shotMap.put(2.6623983102504067, new ShotParams(2625, 2.0, 3.38 - 2.50));
    shotMap.put(2.366682370302446, new ShotParams(2560, 1.950, 3.23 - 2.40));
    shotMap.put(2.21104383698358, new ShotParams(2550, 1.701, 3.18 - 2.26));
    shotMap.put(1.8796353887149986, new ShotParams(2550, 1.293, 3.31 - 2.34));
    shotMap.put(1.3949509529801134, new ShotParams(2550, 0.708, 3.26 - 2.19));

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
      // ShooterSubsystem.shooterSpeedToVelocity(distanceToRPM.get(distance) *
      // (Math.PI / 30.));
      // double xVelocity = Math.cos(launchAngle) * velocity;

      // minTOFDistance;

      // System.out.println("Velocity: " + (distance / TOF) + " & Distance: " +
      // distance);
      System.out.println(
          "Distance: " + distance + " & Velocity: " + (distance / TOF) + " & TOF: " + TOF);

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
          targetingData.robotPosition().getRotation().getRadians()
              + robotToShooter.getAngle().getRadians();
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
}
