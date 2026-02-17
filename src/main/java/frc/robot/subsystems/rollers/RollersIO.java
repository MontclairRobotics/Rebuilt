package frc.robot.subsystems.rollers;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.AngularVelocity;

public interface RollersIO {

	@AutoLog
	public static class RollersIOInputs {
		public double motorVelocity = 0;
		public double appliedVoltage = 0;
		public double temperature = 0;
		public double current = 0;
	}

	/**
	 * Update the IOInputs structure
	 *
	 * @param inputs IO Inputs to update
	 */
	public void updateInputs(RollersIOInputs inputs);

	/**
	 * Applies voltages to the intake motor
	 *
	 * @param voltage voltage to send to the intake motor
	 */
	public void setVoltage(double voltage);

	/** Stops the motor */
	public void stop();

	/**
	 * @return the velocity of the motor.
	 */
	public AngularVelocity getMotorVelocity();
}
