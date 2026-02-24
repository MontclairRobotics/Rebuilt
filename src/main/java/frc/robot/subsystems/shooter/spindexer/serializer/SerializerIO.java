package frc.robot.subsystems.shooter.spindexer.serializer;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.AngularVelocity;

public interface SerializerIO {

	@AutoLog
	public class SerializerIOInputs {
		public boolean motorConnected;

		public AngularVelocity velocity = RotationsPerSecond.zero();
		public AngularVelocity setpointVelocity = RotationsPerSecond.zero();

		public double appliedVoltage = 0.0;
		public double currentDrawAmps = 0.0;
		public double tempCelsius = 0.0;
		public boolean isAtSetpoint = false;
	}

	public void updateInputs(SerializerIOInputs inputs);

	public void setVelocity(AngularVelocity targetVelocity);

	public void setVoltage(double voltage);

	public void stop();

	public boolean isAtSetpoint();

}
