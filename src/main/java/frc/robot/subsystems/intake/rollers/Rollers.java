package frc.robot.subsystems.intake.rollers;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.constants.RollersConstants.INTAKE_VOLTAGE;

public class Rollers extends SubsystemBase {

	private final RollersIO io;
	private final RollersIOInputsAutoLogged inputs = new RollersIOInputsAutoLogged();

	public Rollers(RollersIO io) {
		this.io = io;
	}

	public void intake() {
		io.setVoltage(INTAKE_VOLTAGE);
	}

	public void outtake() {
		io.setVoltage(-INTAKE_VOLTAGE);
	}

	public void stop() {
		io.stop();
	}

	@Override
	public void periodic() {
		io.updateInputs(inputs);
		Logger.processInputs("Intake", inputs);
	}

	public Command stopCommand() {
		return Commands.runOnce(() -> stop(), this);
	}

	public Command intakeCommand() {
		return Commands.run(() -> intake(), this)
			.finallyDo(() -> stop());
	}

	public Command outtakeCommand() {
		return Commands.run(() -> outtake(), this)
			.finallyDo(() -> stop());
	}
}
