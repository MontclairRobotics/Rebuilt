package frc.robot.subsystems.shooter.spindexer.indexer;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.AngularVelocity;

public interface IndexerIO {

	@AutoLog
	public class IndexerIOInputs {
		public boolean motorConnected;

		public AngularVelocity velocity = RotationsPerSecond.zero();
		public AngularVelocity setpointVelocity = RotationsPerSecond.zero();

		public double appliedVoltage = 0.0;
		public double currentDrawAmps = 0.0;
		public double tempCelsius = 0.0;
		public boolean isAtSetpoint = false;
	}

	public void updateInputs(IndexerIOInputs inputs);

	public void setVelocity(AngularVelocity targetVelocity);

	public void setVoltage(double voltage);

	public void stop();

	public boolean isAtSetpoint();
}
