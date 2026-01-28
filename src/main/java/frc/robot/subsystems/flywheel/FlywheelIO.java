package frc.robot.subsystems.flywheel;

import org.littletonrobotics.junction.AutoLog;

public interface FlywheelIO {

	@AutoLog
	public class FlywheelIOInputs {
		public double appliedVoltage = 0.0;
		public double tempCelcius = 0.0;
		public double motorVelocity = 0.0; // rotations per second
		public double flywheelVelocity = 0.0; // rotations per second
	}

	public void updateInputs(FlywheelIOInputs inputs);

	public void setVoltage(double voltage);

	public void stop();

	/**
	 * @return the angular velocity of the <b>motor shaft</b>, in rotations per second
	 */
	public double getMotorVelocity();

	/**
	 * @return the angular velocity of the <b>flywheel</b>, in rotations per second
	 */
	public double getFlywheelVelocity();

	public double getMotorVoltage();

	public double getMotorTemp();
}
