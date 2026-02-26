// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;

import com.ctre.phoenix6.CANBus;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;
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
  public static final CANBus shooterCANBus = new CANBus("Shooter");

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

  // public static class IndexerConstants {
  //   // TODO: set on real robot
  //   public static final int indexerMotorId = 50;
  //   public static final AngularVelocity spindexerSpeed = RPM.of(1500.0);
  // }

  public static class ShooterConstants {
    public static final AngularVelocity launchSpeed = RPM.of(3500.0);
    public static final AngularVelocity minLaunchSpeed = RPM.of(3200.0);
    public static final double flywheelMOI = 0.0011705586; // 28122.783131854398;
    public static final double flywheelGearing = 1;
    public static final int motor1Id = 56;
    public static final int motor2Id = 57;
    public static final Distance flywheelDiameter = Inches.of(2.0);
    public static final int targetingIterations = 20;

    // TODO: set on real robot
    public static final Transform3d positionOnRobot =
        new Transform3d(
            new Translation3d(Inches.of(0.0), Inches.of(5.0), Inches.of(0.0)), new Rotation3d());
    public static final double robotPositionAnticipationSeconds = 0.0;
    public static final double flywheelKP = 0.4;
    public static final double flywheelKI = 0.0;
    public static final double flywheelKD = 0.0;
    public static final double flywheelKV = 0.12;
    public static final int shooterBeamBrakePort = 0;
    public static final int turretMotorId = -1;
    public static final int hoodMotorId = 53;
    public static final double hoodKP = 0;
    public static final double hoodKD = 0;
    public static final double hoodKI = 0;
    public static final Rotation2d maxTurretYaw = new Rotation2d(Math.PI);
    public static final Rotation2d minTurretYaw = new Rotation2d(-Math.PI);
    public static final Rotation2d safeHoodAngle = new Rotation2d();
  }

  public static class IntakeConstants {

    // 4.5

    public static int intakePivotMotor = 52; // made up value
    public static int intakeSpinMotor = 51; // made up value
    public static double intakeMOI = 0.0011705586; // made up value
    public static double intakePivotGearing = 26.84933149230769;
    public static final double intakePivotConversionFactor = 1 / intakePivotGearing;
    public static final double intakeGearing = 4.5;
    public static final double intakeConversionFactor = 1 / intakeGearing;

    public static Distance armLength = Meters.of(1); // made up value
    public static Angle maxAngle = Degrees.of(60); // made up value //60 degrees

    public static double pivotkP = 1.0; // made up value
    public static final double pivotkI = 0;
    public static final double pivotkCos = 0.4;
    public static double pivotkD = 0.00; // made up value
    public static double pivotkV = 0.00; // made up value

    public static double spinkP = 0.0; // made up value
    public static double spinkD = 0.00; // made up value
    public static double spinkV = 0.4; // made up value
    public static AngularVelocity intakeSpeed = RPM.of(1000.0);
    public static Angle deployedAngle = Degrees.of(0);
    public static Angle retractedAngle = Degrees.of(130);
    public static Angle startAngle = Degrees.of(131.76767); // Degrees.of(60.212);
  }

  public static class KickerConstants {
    // TODO set actual CAN Id.
    public static final int motorId = 55;
    // TODO make a conversion factor, or ask a cadder, that works consistently with
    // the internal RPM
    // angular velocity.
    public static final double flywheelGearing = 2.0d;
    public static final double encoderConversionFactor = 1.0 / flywheelGearing;
    public static final AngularVelocity velocitySetpoint = RPM.of(3500d);

    public static final double kP = 0.3d;
    public static final double kI = 0.0d;
    public static final double kD = 0.0d;
    public static final double kS = 0.1d;
    public static final double kV = 0.1224d;

    // SIM SPECIFIC, get cadder for MOI
    public static final double simFlywheelMOI = 0.0011705586;
    public static final AngularVelocity simSetpoint = RPM.of(1550);
    public static final double kVSim = 0.25;
  }

  public static class SpindexerConstants {
    public static final int spindexerMotorId = 50;
    public static final double spindexerMOI = 0.0012; // TBD
    public static final double spindexerGearing = 5d;
    public static final double spindexerConversionFactor = 1 / spindexerGearing;
    public static final double kP = 0.0;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final double kV = 0.00174;
    public static final AngularVelocity spindexerSetSpeed = RPM.of(1500);
  }
}
