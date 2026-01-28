package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static frc.robot.constants.TurretConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.RobotContainer;

public class TurretIOSim implements TurretIO {
	private SingleJointedArmSim sim;
	double appliedVoltage = 0;

	public TurretIOSim() {
		sim = new SingleJointedArmSim(
			DCMotor.getKrakenX60(1),
			GEARING,
			MOMENT_OF_INERTIA,
			LENGTH,
			MIN_ANGLE.in(Radians),
			MAX_ANGLE.in(Radians),
			false,
			0,
			0,
			0
		);
	}

	@Override
	public void updateInputs(TurretIOInputs inputs) {
		// updates the simulator
		sim.setInputVoltage(appliedVoltage);
		sim.update(0.02);

		// updates logged values
		inputs.appliedVoltage = appliedVoltage;
		inputs.motorVelocity = getMotorVelocity();
		inputs.turretVelocity = getTurretVelocity();
		inputs.robotRelativeAngle = getRobotRelativeAngle();
		inputs.fieldRelativeAngle = getFieldRelativeAngle();
	}

	@Override
	public void setVoltage(double voltage) {
		appliedVoltage = MathUtil.clamp(voltage, -12, 12);
	}

	@Override
	public void stop() {
		setVoltage(0);
	}

	@Override
	public double getMotorVelocity() {
		return getTurretVelocity() * GEARING;
	}

	@Override
	public double getTurretVelocity() {
		return RadiansPerSecond.of(sim.getVelocityRadPerSec()).in(RotationsPerSecond);
	}

	@Override
	public double getRobotRelativeAngle() {
		return Radians.of(sim.getAngleRads()).in(Rotations);
	}

	@Override
	public double getFieldRelativeAngle() {
		return RobotContainer.turret.toFieldRelativeAngle(getRobotRelativeAngle());
	}

	@Override
	public void zero() {
		sim.setState(0, 0);
	}
}
