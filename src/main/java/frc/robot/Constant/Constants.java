// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.Constant;

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

    public static final double DRIVE_GEAR_RATIO = 6.88;

    public static final Velocity<VoltageUnit> STEER_RAMP_RATE = null;
    public static final Voltage STEER_STEP_RATE = Units.Volts.of(7);
    public static final Time STEER_TIMEOUT = null;
  }

  public static final class TurretConstants{
    public static final double ELEVATION_GEAR_RATIO = (60.0 / 12.0) * (60.0 / 16.0) * (58.0 / 9.0);
    public static final double ELEVATION_POSITION_COEFFICIENT = 2 * Math.PI * ELEVATION_GEAR_RATIO;
  }
}
