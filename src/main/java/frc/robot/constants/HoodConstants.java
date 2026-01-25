package frc.robot.constants;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;

public class HoodConstants {

  public static final int CAN_ID = 37; // TODO: set
  public static final int ENCODER_PORT = 4; // TODO: set

  public static final Angle MIN_ANGLE = Degrees.of(20); // TODO: set
  public static final Angle MAX_ANGLE = Degrees.of(45); // TODO: set

  public static final AngularVelocity MAX_VELOCITY = RotationsPerSecond.of(3); // TODO: set
  public static final AngularAcceleration MAX_ACCELERATION =
      RotationsPerSecondPerSecond.of(8); // TODO: set

  // for every rotation of the hood the motor shaft rotates this amount
  public static final double GEAR_RATIO = 10.8; // TODO: set
  public static final double MOMENT_OF_INERTIA = 0.03; // TODO: get from CAD

  public static final Angle HOOD_ENCODER_OFFSET = Rotations.of(1); // TODO: get offset
  public static final Distance HOOD_LENGTH = Meters.of(0.305); // TODO: set
  public static final double VOLTAGE_LIMIT = 12; // TODO: set

  public static final double kP = 0;
  public static final double kI = 0;
  public static final double kD = 0;
  public static final double kS = 0;
  public static final double kV = 0;
  public static final double kG = 0;
}
