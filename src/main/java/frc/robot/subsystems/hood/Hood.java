package frc.robot.subsystems.hood;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.util.Tunable;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.Rotations;
import static frc.robot.constants.HoodConstants.*;

public class Hood extends SubsystemBase {
	public HoodIO io;
	private HoodIOInputsAutoLogged inputs = new HoodIOInputsAutoLogged();

	public PIDController pidController;
	public ArmFeedforward feedforward;

	private HoodVisualization visualization;

	public Hood(HoodIO hoodIO) {
		this.io = hoodIO;
		this.visualization = new HoodVisualization();

		Tunable kPTunable = new Tunable("hood kP", kP, (value) -> pidController.setP(value));
		Tunable kITunable = new Tunable("hood kI", kI, (value) -> pidController.setI(value));
		Tunable kDTunable = new Tunable("hood kD", kD, (value) -> pidController.setD(value));
		Tunable kSTunable = new Tunable("hood ks", kS, (value) -> feedforward.setKs(value));
		Tunable kGTunable = new Tunable("hood kg", kG, (value) -> feedforward.setKg(value));
		Tunable kVTunable = new Tunable("hood kv", kV, (value) -> feedforward.setKv(value));

		pidController = new PIDController(kP, kI, kD);
		pidController.setTolerance(PIDTolerance.in(Rotations));
		feedforward = new ArmFeedforward(kS, kG, kV);
	}

	public void applyJoystickInput() {
		double voltage = Math.pow(MathUtil.applyDeadband(RobotContainer.operatorController.getRightY(), 0.04), 3) * 12;
		io.setVoltage(voltage);
	}

	public void setAngle(Supplier<Angle> goalSupplier) {
		setAngle(goalSupplier.get());
	}

	public void setAngle(Angle goal) {
		double pidVoltage = pidController.calculate(io.getAngle().in(Rotations), goal.in(Rotations));
		double feedforwardVoltage = feedforward.calculate(io.getAngle().in(Rotations), 0);
		io.setVoltage(pidVoltage + feedforwardVoltage);
	}

	public boolean atSetpoint() {
		return pidController.atSetpoint();
	}

	public void periodic() {
		io.updateInputs(inputs);
		Logger.processInputs("Hood", inputs);
		visualization.update();
		visualization.log();
	}

	public Command setAngleCommand(Supplier<Angle> supplier) {
		return Commands.run(() -> setAngle(supplier), this);
	}

	public Command setAngleCommand(Angle angle) {
		return Commands.run(() -> setAngle(angle), this).until(() -> atSetpoint());
	}

	public Command joystickCommand() {
		return Commands.run(this::applyJoystickInput, this);
	}
}
