// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.RobotBase;

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

  public static class Field {
    public static final Translation3d hub =
        new Translation3d(Inches.of(181.56), Inches.of(158.84), Inches.of(72));

    public static final Translation3d hubTarget = hub;
  }

  public static class ShooterConstants {
    public static final Translation3d positionOnRobot =
        new Translation3d(Inches.of(0.0), Inches.of(5.0), Inches.of(0.0));
    public static final AngularVelocity launchSpeed = RPM.of(2000.0);
    public static final AngularVelocity minLaunchSpeed = RPM.of(1500.);
    public static final double flywheelMOI = 0.0018687377; // 0.0011705586; // 28122.783131854398;
    public static final double flywheelGearing = 1;
    public static final int motorId = 0;
    public static final Distance flywheelDiameter = Inches.of(2.0);
  }

  public static class SpindexerConstants {
    public static final int spindexerMotorId = 59;
    public static final double spindexerMOI = 0.0012; // TBD
    public static final double spindexerGearing = 1;
    public static final double kP = 0.1;
    public static final double kI = 0;
    public static final double kD = 0.1;
    public static final double spindexerSetSpeed = 5;
  }
}
