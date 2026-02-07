package frc.robot.subsystems.shooter.targeter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import org.littletonrobotics.junction.Logger;

public class TargetingResult {
  public record TargetingResult2d(double pitchRadians, double timeOfFlightSeconds) {}

  public record TargetingResult3d(
      double pitchRadians, double yawRadians, double timeOfFlightSeconds) {

    public void drawTrajectory(
        Translation3d startPosition,
        ChassisSpeeds chassisSpeeds,
        Pose2d shooterPose,
        double launchVelocity,
        String name) {

      Pose3d[] poses = new Pose3d[20];

      double vx =
          chassisSpeeds.vxMetersPerSecond
              + shooterPose.getRotation().plus(new Rotation2d(yawRadians)).getCos()
                  * launchVelocity
                  * Math.cos(this.pitchRadians);
      double vy =
          chassisSpeeds.vyMetersPerSecond
              + shooterPose.getRotation().plus(new Rotation2d(yawRadians)).getSin()
                  * launchVelocity
                  * Math.cos(this.pitchRadians);
      double vz = Math.sin(this.pitchRadians) * launchVelocity;

      double x = startPosition.getX();
      double y = startPosition.getY();
      double z = startPosition.getZ();

      double dt = 0.1;

      for (int i = 0; i < 20; i++) {
        poses[i] = new Pose3d(x, y, z, new Rotation3d());
        vz -= 9.81 * dt;

        x += vx * dt;
        y += vy * dt;
        z += vz * dt;
      }

      Logger.recordOutput("Shooter/Trajectory/" + name, poses);
    }
  }
}
