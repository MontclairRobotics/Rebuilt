package frc.robot.subsystems.shooter.spindexer;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.AngularVelocity;

public interface SpindexerIO {

	@AutoLog
	public class SpindexerIOInputs {
		public double appliedSpinVoltage = 0.0;
		public double appliedIndexVoltage = 0.0;
		public double spinTempCelcius = 0.0;
		public double indexTempCelcius = 0.0;
		public AngularVelocity spinVelocity = RotationsPerSecond.zero();
		public AngularVelocity indexVelocity = RotationsPerSecond.zero();
	}

	public void updateInputs(SpindexerIOInputs inputs);

	public void setSpinVoltage(double voltage);

	public void setIndexVoltage(double voltage);

	public void stopSpin();

	public void stopIndex();
}
