package frc.robot.subsystems.shooter.hood;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.Angle;

public interface HoodIO {
	@AutoLog
	public static class HoodIOInputs {
		public double appliedVoltage;
		public double current;
		public double tempCelsius;
		public double angle; // rotations
		public boolean encoderConnected;
	}

	public void updateInputs(HoodIOInputs inputs);

	public void setVoltage(double voltage);

	/** Gets the angle in Rotations from the relative encoder */
	public Angle getAngle();

	public void stop();

}