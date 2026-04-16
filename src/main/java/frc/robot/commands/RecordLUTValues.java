package frc.robot.commands;

import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem.TurretState;
import frc.robot.util.AllianceFlipUtil;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class RecordLUTValues extends Command {

  private final DoubleSupplier distanceSupplier;
  private final DoubleSupplier angleSupplier;

  private final ShooterSubsystem shooter;

  public RecordLUTValues(
      ShooterSubsystem shooter, DoubleSupplier distanceSupplier, DoubleSupplier angleSupplier) {
    this.distanceSupplier = distanceSupplier;
    this.shooter = shooter;
    this.angleSupplier = angleSupplier;

    addRequirements(shooter);
    SmartDashboard.putNumber("HoodAngle", 0);
    SmartDashboard.putNumber("RPM", 0);
  }

  public RecordLUTValues(ShooterSubsystem shooter, Supplier<Pose2d> poseSupplier) {
    this(
        shooter,
        () -> {
          return poseSupplier
              .get()
              .transformBy(
                  new Transform2d(
                      Constants.ShooterConstants.positionOnRobot.getMeasureX(),
                      Constants.ShooterConstants.positionOnRobot.getMeasureY(),
                      Constants.ShooterConstants.positionOnRobot.getRotation().toRotation2d()))
              .getTranslation()
              .getDistance(AllianceFlipUtil.apply(Constants.Field.hub).toTranslation2d());
        },
        () -> {
          return 0.0;
        });
  }

  @Override
  public void execute() {
    // Julia's secret message
    Logger.recordOutput("Distance", distanceSupplier.getAsDouble());
    shooter.setHoodAngle(SmartDashboard.getNumber("HoodAngle", 0));
    shooter.setTargetAngularVelocity(RPM.of(SmartDashboard.getNumber("RPM", 0)));
    shooter.setTurretState(TurretState.OnTarget);
    shooter.setYaw(new Rotation2d(angleSupplier.getAsDouble()));
  }
}
