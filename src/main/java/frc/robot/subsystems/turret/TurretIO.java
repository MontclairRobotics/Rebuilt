package frc.robot.subsystems.turret;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

public interface TurretIO {

	@AutoLog
	public static class TurretIOInputs {
		public double appliedVoltage;

		// velocities are in rotations per second
		public double motorVelocity;
		public double turretVelocity;

		// angles are in rotations
		public double robotRelativeAngle;
		public double fieldRelativeAngle;
	}

	public void updateInputs(TurretIOInputs inputs);

	public void setVoltage(double volts);

	public void stop();

	/**
	 * @return the angular velocity of the <b>motor shaft</b>, in rotations per second
	 */
	public AngularVelocity getMotorVelocity();

	/**
	 * @return the angular velocity of the <b>turret</b>, in rotations per second
	 */
	public AngularVelocity getTurretVelocity();

	/**
	 * @return the angle of the turret <b>relative to its zero</b>, in rotations
	 */
	public Angle getRobotRelativeAngle();

	/**
	 * @return the angle of the turret <b>relative to the field</b>, in rotations
	 */
	public Angle getFieldRelativeAngle();

	public void zero();
}
