package frc.robot.subsystems.spindexer;

import org.littletonrobotics.junction.AutoLog;

public interface SpindexerIO {

	@AutoLog
	public class SpindexerIOInputs {
		public double spinAppliedVoltage = 0.0;
		public double transAppliedVoltage = 0.0;
		public double spinTempCelcius = 0.0;
		public double transTempCelcius = 0.0;
	}

	public void updateInputs(SpindexerIOInputs inputs);

	public void setSpinVoltage(double currentVoltage);

	public void setTransVoltage(double currentVoltage);

	public void stopSpin();

	public void stopTrans();
}
