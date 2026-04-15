package frc.robot.subsystems.shooter.targeter;

import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import frc.robot.Constants;
import frc.robot.subsystems.shooter.targeter.TargetingResult.TargetingResult3d;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class MechanicalAdvantageTargeter implements Targeter {

  private LoggedNetworkBoolean twistCompensation =
      new LoggedNetworkBoolean("TwistCompensation", true);

  private LoggedNetworkNumber twistCompensationFactor =
      new LoggedNetworkNumber("TwistCompensationFactor", 1.0);

  public static InterpolatingTreeMap<Double, FerryParams> ferryMap =
      new InterpolatingTreeMap<Double, FerryParams>(
          MathUtil::inverseInterpolate, Targeter::ferryInterpolator);

  static {
    ferryMap.put(2.667, new FerryParams(2000, 8.27 - 7.33));

    ferryMap.put(3.835, new FerryParams(2500, 3.76 - 2.79));
    ferryMap.put(5.334, new FerryParams(3000, 7.38 - 6.17));
    ferryMap.put(6.325, new FerryParams(3500, 3.78 - 2.24));

    // ferryMap.put(6.325, new FerryParams(3500, 3.78 - 2.24));
    ferryMap.put(6.5532, new FerryParams(3800, 4.00 - 2.44));
    ferryMap.put(8.5344, new FerryParams(4400, 4.06 - 2.46));

    ferryMap.put(8.8392, new FerryParams(5700, 9.89 - 8.26));

    // regression values
    // ferryMap.put(7.0, new FerryParams(3730.6056, 1.562496));

    // ferryMap.put(7.0, new FerryParams(3730.6056,1.562496));

  }

  private double TOF = 0.0;

  @Override
  public Optional<TargetingResult3d> getShooterTargeting(TargetingData targetingData) {
    double distance = targetingData.target().getNorm();
    Logger.recordOutput("DistancePassedToTargeter", distance);

    Translation2d robotVelocity = targetingData.robotVelocity();

    // if (twistCompensation.get()) {
    //   Translation2d robotToShooter =
    //       Constants.ShooterConstants.positionOnRobot.getTranslation().toTranslation2d();
    //   double twistRadius = robotToShooter.getNorm();
    //   double tangentialVelocity =
    //       twistCompensationFactor.get()
    //           * targetingData.robotOmegaAngularVelocity().in(RadiansPerSecond)
    //           * twistRadius;

    //   // If something is going majorly screwy with our targeting when this switch is
    //   // on, it is almost certainly this godless affront to math in the following ~6
    //   // lines

    //   double velocityAngleRelativeToFieldAxis =
    //       targetingData.robotRotation().getRadians() - robotToShooter.getAngle().getRadians();
    //   Translation2d rotationalVelocity =
    //       new Translation2d(
    //           Math.cos(velocityAngleRelativeToFieldAxis) * tangentialVelocity,
    //           Math.sin(velocityAngleRelativeToFieldAxis) * tangentialVelocity);

    //   robotVelocity = robotVelocity.plus(rotationalVelocity);
    robotVelocity =
        new Translation2d(
            robotVelocity.getX()
                + targetingData.robotOmegaAngularVelocity().in(RadiansPerSecond)
                    * ((Constants.ShooterConstants.positionOnRobot.getY()
                            * targetingData.robotRotation().getCos())
                        - (Constants.ShooterConstants.positionOnRobot.getX()
                            * targetingData.robotRotation().getSin())),
            robotVelocity.getY()
                + targetingData.robotOmegaAngularVelocity().in(RadiansPerSecond)
                    * ((Constants.ShooterConstants.positionOnRobot.getX()
                            * targetingData.robotRotation().getCos())
                        - (Constants.ShooterConstants.positionOnRobot.getY()
                            * targetingData.robotRotation().getSin())));
    // }

    Translation2d lookaheadTarget = targetingData.target();

    for (int i = 0; i <= 40; i++) {
      ShotParams params;
      if (targetingData.shouldFerry()) {
        params = ferryMap.get(lookaheadTarget.getNorm()).getShotParams();
      } else {
        params = EeshwarkTargeter.shotMap.get(lookaheadTarget.getNorm());
      }

      lookaheadTarget = targetingData.target().minus(robotVelocity.times(params.TOF()));
      //   lookaheadTarget.rotateBy(
      //       new Rotation2d(
      //           targetingData.robotOmegaAngularVelocity().in(RadiansPerSecond)
      //               * params.TOF()
      //               * twistCompensationFactor.get()));
    }

    Translation2d shotDirection = lookaheadTarget.div(lookaheadTarget.getNorm());

    if (shotDirection.getSquaredNorm() == 0
        || Double.isNaN(shotDirection.getX())
        || Double.isNaN(shotDirection.getY())) {
      return Optional.empty();
    }

    Rotation2d shotYaw = shotDirection.getAngle();
    double fieldRelativeYaw = shotYaw.getRadians();
    ShotParams params = EeshwarkTargeter.shotMap.get(lookaheadTarget.getNorm());
    TOF = params.TOF();

    return Optional.of(
        new TargetingResult3d(params.hoodPosition(), params.RPM(), fieldRelativeYaw, params.TOF()));
  }

  public double getTOF() {
    return TOF;
  }
}
