package frc.robot.subsystems.intake.rollers;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.AngularVelocity;

public interface RollersIO {

	@AutoLog
	public static class RollersIOInputs {
		public boolean motorConnected;

		public AngularVelocity velocity = RotationsPerSecond.zero();
		public AngularVelocity setpointVelocity = RotationsPerSecond.zero();

		public double appliedVoltage = 0.0;
		public double currentDrawAmps = 0.0;
		public double tempCelsius = 0.0;
		public boolean isAtSetpoint = false;
	}

	public void updateInputs(RollersIOInputs inputs);

	public void setVelocity(AngularVelocity velocity);

	public void setVoltage(double voltage);

	public void stop();

	public boolean isAtSetpoint();
}
