package frc.robot.subsystems.shooter.spindexer.serializer;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static frc.robot.constants.SerializerConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class SerializerIOSim implements SerializerIO {

	private final FlywheelSim sim;
	private double appliedVoltage;

	private PIDController pidController;
	private SimpleMotorFeedforward feedforward;

	public SerializerIOSim() {
		sim = new FlywheelSim(
			LinearSystemId.createFlywheelSystem(
				DCMotor.getKrakenX60Foc(1),
				MOMENT_OF_INERTIA,
				GEARING
			),
			DCMotor.getKrakenX60Foc(CAN_ID), null
		);

		pidController = new PIDController(
			kP, 0, kD
		);

		pidController.setTolerance(VELOCITY_TOLERANCE.in(RotationsPerSecond));
		feedforward = new SimpleMotorFeedforward(kS, kV);
	}

	@Override
	public void updateInputs(SerializerIOInputs inputs) {
		sim.setInputVoltage(appliedVoltage);
		sim.update(0.02);

		inputs.motorConnected = true;

		inputs.velocity = RadiansPerSecond.of(sim.getAngularVelocityRadPerSec());
		inputs.setpointVelocity = RotationsPerSecond.of(pidController.getSetpoint());

		inputs.appliedVoltage = appliedVoltage;
		inputs.currentDrawAmps = sim.getCurrentDrawAmps();
		inputs.tempCelsius = 0;
		inputs.isAtSetpoint = isAtSetpoint();
	}

	@Override
	public void setVelocity(AngularVelocity targetVelocity) {
		double pidOutput = pidController.calculate(
            RadiansPerSecond.of(sim.getAngularVelocityRadPerSec()).in(RotationsPerSecond),
            targetVelocity.in(RotationsPerSecond)
        );
		double ffOutput = feedforward.calculate(targetVelocity.in(RotationsPerSecond));
		double totalOutput = pidOutput + ffOutput;
        appliedVoltage = MathUtil.clamp(totalOutput, -RobotController.getBatteryVoltage(), RobotController.getBatteryVoltage());
	}

	@Override
	public void setVoltage(double voltage) {
		appliedVoltage = voltage;
	}

	@Override
	public void stop() {
		appliedVoltage = 0;
	}

	@Override
	public boolean isAtSetpoint() {
		return pidController.atSetpoint();
	}
}
