package frc.robot.constants;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;

public class HoodConstants {
	
	public static final double MOTOR_ID = 37; //TODO: set
	public static final Angle MIN_ANGLE = Degrees.of(20);
	public static final Angle MAX_ANGLE = Degrees.of(45);
	public static final AngularVelocity MAX_VELOCITY = RotationsPerSecond.of(1);
	public static final AngularAcceleration MAX_ACCELERATION = RotationsPerSecondPerSecond.of(5);

	// for every rotation of the hood the motor shaft rotates this amount
	public static final double GEAR_RATIO = 10; //TODO: set
	public static final double MOMENT_OF_INERTIA = 10; //TODO: get from CAD

}
