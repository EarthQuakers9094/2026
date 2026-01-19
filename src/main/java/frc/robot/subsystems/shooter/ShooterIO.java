package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
  @AutoLog
  public static class ShooterIOInputs {
    Rotation2d currentPitch = new Rotation2d();
  }

  public default void updateInputs(ShooterIOInputs inputs) {}

  public default void shoot() {}

  public default void setPitch(Rotation2d pitch) {}
}
