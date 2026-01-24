package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.AngularVelocity;
import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
  @AutoLog
  public static class ShooterIOInputs {
    public AngularVelocity shooterSpeed = RadiansPerSecond.of(0.);

    public Rotation2d currentPitch = new Rotation2d();
  }

  public default void updateInputs(ShooterIOInputs inputs) {}

  public default void setPitch(Rotation2d pitch) {}

  public default void spinUpShooter() {}
  ;

  public default void stopShooter() {}
  ;

  public default void startIndexing() {}
  ;

  public default void stopIndexing() {}
  ;
}
