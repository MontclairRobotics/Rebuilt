package frc.robot.subsystems.flywheel;

import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.AutoLog;

public interface FlywheelIO {

	@AutoLog
	public class FlywheelIOInputs {
		public double appliedVoltage = 0.0;
		public double velocity = 0.0; // rps
		public double tempCelcius = 0.0;
		public double velocitySetpoint = 0.0;
	}

	public void updateInputs(FlywheelIOInputs inputs);

	public void setVoltage(double currentVoltage);

	public void stop();

	public void setVelocityRPS(double targetVelocity);

	public void setVelocityRPS(DoubleSupplier targetVelocitySupplier);

	public boolean atSetPoint();
}
