package frc.robot.subsystems.rollers;

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

	// --------------------------COMMANDS--------------------------

	public Command stopCommand() {
		return Commands.runOnce(this::stop, this);
	}

	// power intake
	public Command intakeCommand() {
		return Commands.run(this::intake, this).finallyDo(this::stop);
	}

	// used for unjamming
	public Command outtakeCommand() {
		return Commands.run(this::outtake, this);
	}
}
