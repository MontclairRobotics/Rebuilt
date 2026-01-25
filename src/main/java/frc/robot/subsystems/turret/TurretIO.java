package frc.robot.subsystems.turret;

import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.AutoLog;

public interface TurretIO {
	
	@AutoLog
	public static class TurretIOInputs {
		public double velocity;
		public double appliedVoltage;
		public double robotRelativeAngle; // angles are in rotations
		public double robotRelativeAngleSetpoint;
		public double fieldRelativeAngle;
		public double fieldRelativeAngleSetpoint;
	}

	public void updateInputs(TurretIOInputs inputs);

	public double getRobotRelativeAngle(); // encoders are relative

	public double getFieldRelativeAngle();

	public boolean atSetpoint();

	public double wrapAngleSetpoint(double angle);

	public void zeroRelativeEncoder();

	public void setRobotRelativeAngle(double angle);

	public void setRobotRelativeAngle(DoubleSupplier supplier);

	public void setFieldRelativeAngle(double angle);

	public void setFieldRelativeAngle(DoubleSupplier supplier);

	public void setVoltage(double volts); // maybe double?

	public void stop(); // also command w/ binding, especially for testing
}
