package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static frc.robot.constants.TurretConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
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
			LENGTH.in(Meters),
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
		inputs.motorVelocity = getMotorVelocity().in(RotationsPerSecond);
		inputs.turretVelocity = getTurretVelocity().in(RotationsPerSecond);
		inputs.robotRelativeAngle = getRobotRelativeAngle().in(Rotations);
		inputs.fieldRelativeAngle = getFieldRelativeAngle().in(Rotations);
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
	public AngularVelocity getMotorVelocity() {
		return getTurretVelocity().times(GEARING);
	}

	@Override
	public AngularVelocity getTurretVelocity() {
		return RadiansPerSecond.of(sim.getVelocityRadPerSec());
	}

	@Override
	public Angle getRobotRelativeAngle() {
		return Radians.of(sim.getAngleRads());
	}

	@Override
	public Angle getFieldRelativeAngle() {
		return RobotContainer.turret.toFieldRelativeAngle(getRobotRelativeAngle());
	}

	@Override
	public void zero() {
		sim.setState(0, 0);
	}
}
