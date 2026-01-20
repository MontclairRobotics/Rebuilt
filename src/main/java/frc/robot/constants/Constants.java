// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.constants;

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

  //TODO: make this an enum list???
  public class IntakeConstants {
    public static final double INTAKE_FUEL_SPEED = 0.0; //TODO: set this number
    public static final double MAX_CURRENT = 0.0; //TODO: set this number
    public static final double OUTTAKE_FUEL_SPEED = 0.0; //TODO: set this number
    public static final double INTAKE_STALL_CURRENT = 0.0; //TODO: set this number
    public static final int MOTOR_ID = -1; //TODO: set the device id
    public static final 
    
  }
}
