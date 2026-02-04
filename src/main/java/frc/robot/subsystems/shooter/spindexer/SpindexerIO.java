package frc.robot.subsystems.shooter.spindexer;

import org.littletonrobotics.junction.AutoLog;

public interface SpindexerIO {

	@AutoLog
	public class SpindexerIOInputs {
		public double appliedVoltage = 0.0;
		public double velocity = 0.0; // rps
		public double tempCelcius = 0.0;
	}

	public void updateInputs(SpindexerIOInputs inputs);

	public void setVoltage(double currentVoltage);

	public void stop();

}
