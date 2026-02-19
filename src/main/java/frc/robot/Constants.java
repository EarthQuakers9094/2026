// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import java.nio.file.Path;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static class Camera {

    // TODO: set on real robot
    public static final double linearStdDev = 1.0;
    public static final double angularStdDev = 1.0;
  }

  public static class Field {
    public static final Translation3d hub =
        new Translation3d(Inches.of(181.56), Inches.of(158.84), Inches.of(72));

    public static final Translation3d hubTarget =
        hub.plus(new Translation3d(Inches.of(5), Inches.of(0), Inches.of(0)));

    public static final AprilTagFieldLayout aprilTagLayout = getFieldLayout();

    private static AprilTagFieldLayout getFieldLayout() {
      try {
        return new AprilTagFieldLayout(
            Path.of(Filesystem.getDeployDirectory().getPath(), "apriltags", "2026-official.json"));
      } catch (Exception e) {
        throw new RuntimeException(e);
      }
    }

    public static final double fieldLength = aprilTagLayout.getFieldLength();
    public static final double fieldWidth = aprilTagLayout.getFieldWidth();

    public static final Distance allianceZoneWidth = Inches.of(182.11);

    public static final Distance trenchOrigin = Inches.of(158.61);

    public static final Distance trenchWidth = Inches.of(23.5 * 2);

    public static final Distance trenchHeight = Inches.of(49.48);
  }

  public static class IndexerConstants {
    // TODO: set on real robot
    public static final int indexerMotorId = 0;
  }

  public static class ShooterConstants {
    public static final AngularVelocity launchSpeed = RPM.of(1800.0);
    public static final AngularVelocity minLaunchSpeed = RPM.of(1700.);
    public static final double flywheelMOI = 0.0011705586; // 28122.783131854398;
    public static final double flywheelGearing = 1;
    public static final int motor1Id = 0;
    public static final int motor2Id = 1;
    public static final Distance flywheelDiameter = Inches.of(2.0);
    public static final int targetingIterations = 20;

    // TODO: set on real robot
    public static final Transform3d positionOnRobot =
        new Transform3d(
            new Translation3d(Inches.of(0.0), Inches.of(5.0), Inches.of(0.0)), new Rotation3d());
    public static final double robotPositionAnticipationSeconds = 0.0;
    public static final double flywheelKP = 0.0;
    public static final double flywheelKI = 0.0;
    public static final double flywheelKD = 0.0;
    public static final double flywheelKV = 0.0;
    public static final int shooterBeamBrakePort = 0;
    public static final int turretMotorId = 0;
    public static final int hoodMotorId = 0;
    public static final double hoodKP = 0;
    public static final double hoodKD = 0;
    public static final double hoodKI = 0;
    public static final Rotation2d maxTurretYaw = new Rotation2d(Math.PI);
    public static final Rotation2d minTurretYaw = new Rotation2d(-Math.PI);
    public static final Rotation2d safeHoodAngle = new Rotation2d();
  }

  public static class KickerConstants {
    // TODO set actual CAN Id.
    public static final int motorId = -1;
    public static final double encoder_conversion_factor = 1.0d;
    public static final double activeVelocity = 1;

    public static final double kP = 0.2d;
    public static final double kI = 0.0d;
    public static final double kD = 0.0d;
    public static final double kS = 0.0d;
    public static final double kV = 0.0d;

    // SIM SPECIFIC
    public static final double simFlywheelMOI = 1.0d;
    public static final double simFlywheelGearing = 1.0d;
  }
}
