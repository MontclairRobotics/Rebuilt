package frc.robot.subsystems.flywheel;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.constants.FlywheelConstants;
import java.util.function.DoubleSupplier;

public class FlywheelIOSim implements FlywheelIO {

	private PIDController pidController;
	private SimpleMotorFeedforward motorFeedForward;

	private FlywheelSim sim =
		new FlywheelSim(
			LinearSystemId.createFlywheelSystem(
				DCMotor.getKrakenX60(1),
				FlywheelConstants.MOMENT_OF_INERTIA,
				FlywheelConstants.GEARING),
			DCMotor.getKrakenX60(1),
			0.0
		);

	public FlywheelIOSim() {
		pidController =
			new PIDController(
				FlywheelConstants.SLOT0_CONFIGS.kP,
				FlywheelConstants.SLOT0_CONFIGS.kI,
				FlywheelConstants.SLOT0_CONFIGS.kD
			);
		motorFeedForward =
			new SimpleMotorFeedforward(
				FlywheelConstants.SLOT0_CONFIGS.kS, FlywheelConstants.SLOT0_CONFIGS.kV
			);
	}

	public void updateInputs(FlywheelIOInputs inputs) {
		inputs.appliedVoltage = sim.getInputVoltage();
		inputs.velocity = (sim.getAngularVelocityRadPerSec() / 2 * Math.PI);
		inputs.tempCelcius = 0; // TODO: maybe figure out how to do this?
		inputs.velocitySetpoint = pidController.getSetpoint();
	}

	@Override
	public void setVoltage(double currentVoltage) {
		sim.setInputVoltage(currentVoltage);
	}

	public void stop() {
		setVoltage(0);
	}

	public void setVelocityRPS(double targetVelocity) {
		// converted radians/sec -> rotations.sec
		double pidOutput = pidController.calculate((sim.getAngularVelocity()).in(RotationsPerSecond), targetVelocity);
		double ffVolts = motorFeedForward.calculate(targetVelocity);
		double totalOutput = pidOutput + ffVolts;
		sim.setInputVoltage(MathUtil.clamp(totalOutput, -12.0, 12.0));
	}

	public void setVelocityRPS(DoubleSupplier targetVelocitySupplier) {
		setVelocityRPS(targetVelocitySupplier.getAsDouble());
	}

	public boolean atSetPoint() {
		return pidController.atSetpoint();
	}
}
