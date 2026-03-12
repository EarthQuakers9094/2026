package frc.robot.commands;

import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.shooter.targeter.Targeter;
import frc.robot.subsystems.shooter.targeter.Targeter.TargetingData;
import frc.robot.subsystems.shooter.targeter.TargetingResult.TargetingResult3d;
import frc.robot.util.AllianceFlipUtil;
import java.util.Optional;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class ShooterTrackTarget extends Command {

  private final ShooterSubsystem shooterSubsystem;
  private final Targeter targeter;
  private final Supplier<Pose2d> robotPositionSupplier;
  private final Supplier<ChassisSpeeds> chassisSpeedsSupplier;
  private final Supplier<Translation3d> targetSupplier;
  private final boolean shouldFlipTarget;

  public ShooterTrackTarget(
      ShooterSubsystem shooterSubsystem,
      Supplier<Pose2d> robotPositionSupplier,
      Supplier<ChassisSpeeds> chassisSpeedsSupplier,
      Targeter targeter,
      Translation3d target) {

    this(shooterSubsystem, robotPositionSupplier, chassisSpeedsSupplier, targeter, target, false);
  }

  public ShooterTrackTarget(
      ShooterSubsystem shooterSubsystem,
      Supplier<Pose2d> robotPositionSupplier,
      Supplier<ChassisSpeeds> chassisSpeedsSupplier,
      Targeter targeter,
      Translation3d target,
      boolean shouldFlipTarget) {
    this(
        shooterSubsystem,
        robotPositionSupplier,
        chassisSpeedsSupplier,
        targeter,
        () -> target,
        shouldFlipTarget);
  }

  public ShooterTrackTarget(
      ShooterSubsystem shooterSubsystem,
      Supplier<Pose2d> robotPositionSupplier,
      Supplier<ChassisSpeeds> chassisSpeedsSupplier,
      Targeter targeter,
      Supplier<Translation3d> targetSupplier,
      boolean shouldFlipTarget) {
    this.shooterSubsystem = shooterSubsystem;
    this.targeter = targeter;
    this.robotPositionSupplier = robotPositionSupplier;
    this.chassisSpeedsSupplier = chassisSpeedsSupplier;
    this.targetSupplier = targetSupplier;
    this.shouldFlipTarget = shouldFlipTarget;

    SmartDashboard.putNumber("HoodAngle", 0.0);

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

    Translation3d flippedTarget =
        shouldFlipTarget ? AllianceFlipUtil.apply(targetSupplier.get()) : targetSupplier.get();

    Translation2d shooterToTarget =
        flippedTarget.toTranslation2d().minus(anticipatedShooterPosition.getTranslation());
    double distanceToTarget = shooterToTarget.getNorm();

    Logger.recordOutput("Target", flippedTarget);

    // AngularVelocity idealShooterSpeed =
    // ShooterSubsystem.getIdealShooterSpeed(distanceToTarget);
    // double targetPitch = ShooterSubsystem.getIdealPitch(distanceToTarget);
    // shooterSubsystem.setTargetAngularVelocity(idealShooterSpeed);
    // Logger.recordOutput("Terri", null);
    // System.out.println("About to set thing");
    Logger.recordOutput("Setting Pitch", Timer.getFPGATimestamp());

    // double shooterSpeed =
    // shooterSubsystem.getShooterSpeed().in(RadiansPerSecond);
    // if (!shooterSubsystem.isSpunUp()) {
    // shooterSpeed = idealShooterSpeed.in(RadiansPerSecond);
    // }

    Optional<TargetingResult3d> maybeTargetingResult =
        targeter.getShooterTargeting(
            new TargetingData(
                shooterToTarget,
                flippedTarget.getMeasureZ(),
                new Translation2d(
                    chassisSpeeds.vxMetersPerSecond * (RobotBase.isReal() ? 1.0 : -1.0),
                    chassisSpeeds.vyMetersPerSecond * (RobotBase.isReal() ? 1.0 : -1.0)) // I
                // cannot
                // claim
                // to
                // understand
                // why
                // i
                // need
                // to
                // do
                // this,
                // but
                // i
                // do.
                ));
    if (maybeTargetingResult.isPresent()) {
      TargetingResult3d targetingResult = maybeTargetingResult.get();
      // Logger.recordOutput("IdealPitch", targetingResult.pitchRadians());
      shooterSubsystem.setTargetAngularVelocity(RPM.of(targetingResult.targetRPM()));
      // shooterSubsystem.setTargetAngularVelocity(RPM.of(SmartDashboard.getNumber("RPM",
      // 0)));
      // shooterSubsystem.setHoodAngle(targetingResult.hoodAngle());
      // shooterSubsystem.setHoodAngle(targetingResult.hoodAngle());

      shooterSubsystem.setYaw(
          new Rotation2d(targetingResult.yawRadians()).minus(robotPosition.getRotation()));
              shooterSubsystem.setPitch(new Rotation2d(targetingResult.pitchRadians()));

      // shooterSubsystem.setPitch(new Rotation2d(targetingResult.pitchRadians()));
      // if (targetingResult.pitchRadians() >= Math.PI / 2.) {
      // DriverStation.reportError(
      // "Targeting pitch result ("
      // + targetingResult.pitchRadians()
      // + ") is outside of physically possible range.",
      // true);
      // // TODO: Not sure what to do in this case... we're asking just too much of
      // our
      // // shooter
      // } else {
      // // shooterSubsystem.setPitch(new Rotation2d(targetingResult.pitchRadians()));
      // }
    } else {
      DriverStation.reportError("AAAAAAAAAAAHA", false);
    }
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
