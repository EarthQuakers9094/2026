package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.Constants;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
  @AutoLog
  public static class ShooterIOInputs {
    public AngularVelocity shooterSpeed = RPM.of(0.);
    public boolean isFuelInShooter = false;
  }

  public default void updateInputs(ShooterIOInputs inputs) {}

  public default void setPitch(Rotation2d pitch) {}

  public default void setYaw(Rotation2d yaw) {}

  public default void setVelocitySetpoint(AngularVelocity speed) {}

  public default void startIndexing() {}

  public default void stopIndexing() {}

  public default void retractHood() {
    this.setPitch(Constants.ShooterConstants.safeHoodAngle);
  }
}
