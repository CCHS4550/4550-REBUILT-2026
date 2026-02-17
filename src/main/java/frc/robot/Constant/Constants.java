// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.Constant;

import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;
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

  public static final class SysIdConstants {
    public static final Velocity<VoltageUnit> TRANSLATION_RAMP_RATE = null;
    public static final Voltage TRANSLATION_STEP_RATE = Units.Volts.of(7);
    public static final Time TRANSLATION_TIMEOUT = Units.Seconds.of(5);

    /* This is in radians per second², but SysId only supports "volts per second" */
    public static final Velocity<VoltageUnit> ROTATION_RAMP_RATE =
        Units.Volts.of(Math.PI / 6).per(Units.Second);
    /* This is in radians per second, but SysId only supports "volts" */
    public static final Voltage ROTATION_STEP_RATE = Units.Volts.of(Math.PI);
    public static final Time ROTATION_TIMEOUT = Units.Seconds.of(5);

    public static final Velocity<VoltageUnit> STEER_RAMP_RATE = null;
    public static final Voltage STEER_STEP_RATE = Units.Volts.of(7);
    public static final Time STEER_TIMEOUT = null;

    public static final double DRIVE_GEAR_RATIO = 6.38;
  }

  public static final class TurretConstants {
    public static final double ELEVATION_GEAR_RATIO =
        (60.0 / 12.0) * (60.0 / 16.0) * (58.0 / 9.0); // fill
    public static final double ELEVATION_POSITION_COEFFICIENT = 2 * Math.PI * ELEVATION_GEAR_RATIO;

    public static final double ROTATION_GEAR_RATIO = 0.0;
    public static final double ROTATION_POSITION_COEFFICIENT = 0.0;

    public static final double SHOOTER_MAX_RADIANS_PER_SEC = 0.0;

    public static final double TURRET_HEIGHT_METERS = 0.0;

    public static final Transform2d TURRET_TRANSFORM = new Transform2d();

    public static final double MAX_BALL_HEIGHT_METERS = 5.4864;

    public static final double FORWARD_ROTATION_LIMIT_RADIANS = 0.0;

    public static final double BACKWARDS_ROTATION_LIMIT_RADIANS = 0.0;
  }

  public static final class IntakeConstants {
    public static final double EXTENSION_GEAR_RATIO =
        (60.0 / 12.0) * (60.0 / 16.0) * (58.0 / 9.0); // fill
    public static final double EXTENSION_POSITION_COEFFICIENT = 2 * Math.PI * EXTENSION_GEAR_RATIO;
  }

  public static final class LowerChassisConstants {
    public static final double KICKER_GEAR_RATIO =
        (60.0 / 12.0) * (60.0 / 16.0) * (58.0 / 9.0); // fill
    public static final double KICKER_POSITION_COEFFICIENT = 2 * Math.PI * KICKER_GEAR_RATIO;
  }

  public static final class VisionConstants {
    public static final double linearStdDevBaseline = 0.02;
    public static final double angularStdDevBaseline = 0.06;
    public static final double maxAmbiguity = 0.1;
    public static final double maxZError = 0.75;
  }

  public static final class ShooterCalculationConstants {
    public static final double GRAVITATION_CONSTANT = 9.81;
    public static final double TIME_DELAY = 0.03;
    public static final double GEOMETRY_VELOCITY = 6.7;
    public static final double TURRET_HEIGHT = 0.12;
    // public static final Pose2d HUB_POSITION = new Pose2d(4.625594,4.034663,new Rotation2d());

  }
}
