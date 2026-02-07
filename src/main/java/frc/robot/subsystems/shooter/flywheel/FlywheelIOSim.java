package frc.robot.subsystems.shooter.flywheel;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static frc.robot.constants.FlywheelConstants.*;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class FlywheelIOSim implements FlywheelIO {

	private FlywheelSim sim;

	public FlywheelIOSim() {
		sim = new FlywheelSim(
			LinearSystemId.createFlywheelSystem(
				DCMotor.getKrakenX60(2),
				MOMENT_OF_INERTIA,
				GEARING
			),
			DCMotor.getKrakenX60(2),
			0.0
		);
	}

	@Override
	public void updateInputs(FlywheelIOInputs inputs) {
		inputs.appliedVoltage = getMotorVoltage();
		inputs.tempCelcius = getMotorTemp();
		inputs.motorVelocity = getMotorVelocity().in(RotationsPerSecond);
		inputs.flywheelVelocity = getFlywheelVelocity().in(RotationsPerSecond);
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
	public AngularVelocity getMotorVelocity() {
		return RotationsPerSecond.of(getFlywheelVelocity().in(RotationsPerSecond) * GEARING);
	}

	@Override
	public AngularVelocity getFlywheelVelocity() {
		return sim.getAngularVelocity();
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