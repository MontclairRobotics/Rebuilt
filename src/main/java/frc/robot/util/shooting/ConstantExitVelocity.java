package frc.robot.util.shooting;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;

public class ConstantExitVelocity {
	public double vyTurret; // radial velocity component of the turret relative to the goal
	public double vxTurret; // velocity component along the axis towards the goal
	public double v0; // upward velocity of the ball, CONSTANT
	public double tGoal; // time it take for the ball to travel into the goal, CONSTANT
	public double deltaX; // distance from turret to goal

	public double v0x, v0y, v0z; // velocity vectors of the ball when it comes out of the turret

	public void updateInputs() {
		// TODO: make this get vyTurret, vxTurret, and deltaX from their respective classes
		v0x = (deltaX / tGoal) - vxTurret;
		v0y = -vyTurret;
		v0z = Math.sqrt(v0 * v0 - v0x * v0x - v0y * v0y);
		// TODO: set the values of tGoal and v0
	}

	public Angle getHoodAngle() {
		double shooterAngleRadians = Math.acos(Math.sqrt(v0x * v0x + v0y * v0y) / v0);
		return Radians.of(
			Math.PI / 2
				- shooterAngleRadians); // Subtracts from pi/2 radians to convert from shooter angle to
		// hood angle
	}

	public Angle getTurretOffset() {
		return Radians.of(Math.atan(v0y / v0x));
	}

	public LinearVelocity getEjectSpeed() {
		return MetersPerSecond.of(v0);
	}

	public AngularVelocity getShooterAngularVelocity() {
		return RotationsPerSecond.of(0); // TODO: Make this actually work
	}
}
