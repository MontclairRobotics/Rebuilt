package frc.robot.subsystems.flywheel;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static frc.robot.constants.FlywheelConstants.*;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class FlywheelIOSim implements FlywheelIO {

	private FlywheelSim sim;

	public FlywheelIOSim() {
		sim = new FlywheelSim(
			LinearSystemId.createFlywheelSystem(
				DCMotor.getKrakenX60(1),
				MOMENT_OF_INERTIA,
				GEARING
			),
			DCMotor.getKrakenX60(1),
			0.0
		);
	}

	@Override
	public void updateInputs(FlywheelIOInputs inputs) {
		inputs.appliedVoltage = getMotorVoltage();
		inputs.tempCelcius = getMotorTemp();
		inputs.motorVelocity = getMotorVelocity();
		inputs.flywheelVelocity = getFlywheelVelocity();
	}

	@Override
	public void setVoltage(double voltage) {
		sim.setInputVoltage(voltage);
	}

	@Override
	public void stop() {
		setVoltage(0);
	}

	@Override
	public double getMotorVelocity() {
		return getFlywheelVelocity() * GEARING;
	}

	@Override
	public double getFlywheelVelocity() {
		return sim.getAngularVelocity().in(RotationsPerSecond);
	}

	@Override
	public double getMotorVoltage() {
		return sim.getInputVoltage();
	}

	@Override
	public double getMotorTemp() {
		return 0; 
	}
}
