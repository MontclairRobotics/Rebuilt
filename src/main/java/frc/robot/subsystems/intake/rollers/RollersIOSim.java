package frc.robot.subsystems.intake.rollers;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import edu.wpi.first.units.measure.AngularVelocity;

public class RollersIOSim implements RollersIO {

	private double appliedVoltage;

	public RollersIOSim() {}

	@Override
	public void updateInputs(RollersIOInputs inputs) {
		inputs.appliedVoltage = appliedVoltage;
		inputs.motorVelocity = 0;
		inputs.current = 0;
		inputs.temperature = 0;
	}

	@Override
	public void setVoltage(double voltage) {
		appliedVoltage = voltage;
	}

	@Override
	public void stop() {
		setVoltage(0);
	}

	@Override
	/**
	 * @return the angular velocity of the rollers
	 */
	public AngularVelocity getMotorVelocity() {
		return RotationsPerSecond.zero();
	}
}
