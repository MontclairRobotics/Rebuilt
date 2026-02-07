package frc.robot.subsystems.shooter.spindexer;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.AngularVelocity;

public interface SpindexerIO {

	@AutoLog
	public class SpindexerIOInputs {
		public double spinAppliedVoltage = 0.0;
		public double indexAppliedVoltage = 0.0;
		public double spinTempCelcius = 0.0;
		public double indexTempCelcius = 0.0;
		public AngularVelocity spinVelocity = AngularVelocity.ofBaseUnits(0, RotationsPerSecond);
		public AngularVelocity indexVelocity = AngularVelocity.ofBaseUnits(0, RotationsPerSecond);
	}

	public void updateInputs(SpindexerIOInputs inputs);

	public void setSpinVoltage(double currentVoltage);

	public void setIndexVoltage(double currentVoltage);

	public void stopSpin();

	public void stopIndex();
}