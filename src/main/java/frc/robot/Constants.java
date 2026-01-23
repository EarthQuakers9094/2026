// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
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
    public static final LinearVelocity launchSpeed = MetersPerSecond.of(10.0);
    public static final AngularVelocity minLaunchSpeed = RotationsPerSecond.of(100.);
    public static final double flywheelMOI = 28122.783131854398;
    public static final double flywheelGearing = 1;
  }
}
