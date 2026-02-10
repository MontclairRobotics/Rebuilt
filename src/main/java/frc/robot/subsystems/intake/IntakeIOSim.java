package frc.robot.subsystems.intake;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.AngularVelocity;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import static frc.robot.constants.IntakeConstants.GEARING;
import static frc.robot.constants.IntakeConstants.MOMENT_OF_INERTIA;

public class IntakeIOSim implements IntakeIO {
		
	private FlywheelSim sim;
	private double appliedVoltage;

	public IntakeIOSim() {
		sim = new FlywheelSim(
			LinearSystemId.createFlywheelSystem(DCMotor.getKrakenX60(1), MOMENT_OF_INERTIA, GEARING),
			DCMotor.getKrakenX60(1),
			0.0
		);
	}

	@Override
	public void updateInputs(IntakeIOInputs inputs) {

		sim.setInputVoltage(appliedVoltage);
		sim.update(0.02);

		inputs.appliedVoltage = sim.getInputVoltage();
		inputs.velocity = getVelocity().in(RotationsPerSecond);
		inputs.current = sim.getCurrentDrawAmps();
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
	public AngularVelocity getVelocity() {
		return sim.getAngularVelocity();
	}
}
