package frc.robot.subsystems.intake.pivot;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import org.littletonrobotics.junction.AutoLog;

import com.ctre.phoenix6.signals.NeutralModeValue;

public interface PivotIO {

	@AutoLog
	public static class PivotIOInputs {
		public boolean motorConnected = false;

        public double appliedVoltage = 0.0;
        public double currentDrawAmps = 0.0;
        public double tempCelcius = 0.0;

        public Angle angle = Rotations.zero();
        public Angle angleSetpoint = Rotations.zero();
        public AngularVelocity velocity = RotationsPerSecond.zero();

        public boolean isAtSetpoint = false;
	}

	public void updateInputs(PivotIOInputs inputs);

	public void setVoltage(double voltage);

	public void stop();

	/**
	 * @return the Angle of the pivot
	 */
	public Angle getAngle();

	public void setAngle(Angle angle);

	public boolean isAtSetpoint();

	public void resetEncoderPosition();

	public void setGains(double kP, double kD, double kS, double kG);

	public void setMotionMagic(AngularVelocity velocity, AngularAcceleration acceleration, double jerk);

	public void setNeutralMode(NeutralModeValue value);
}
