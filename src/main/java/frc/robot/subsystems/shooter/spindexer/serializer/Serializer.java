package frc.robot.subsystems.shooter.spindexer.serializer;

import static frc.robot.constants.SerializerConstants.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.shooter.spindexer.SpindexerIOInputsAutoLogged;

import org.littletonrobotics.junction.Logger;

public class Serializer extends SubsystemBase {

	private SerializerIO io;
	private SpindexerIOInputsAutoLogged inputs = new SpindexerIOInputsAutoLogged();

	public Serializer(SerializerIO io) {
		this.io = io;
	}

	@Override
	public void periodic() {
		io.updateInputs(inputs);
		Logger.processInputs("Spindexer", inputs);
	}

	public void spin() {
		io.setSpinVoltage(SPIN_VOLTAGE);
		io.setIndexVoltage(INDEX_VOLTAGE);
	}

	public void reverseSpin() {
		io.setSpinVoltage(-SPIN_VOLTAGE);
		io.setIndexVoltage(-INDEX_VOLTAGE);
	}

	public void stop() {
		io.stopSpin();
		io.stopIndex();
	}

	public Command stopCommand() {
		return Commands.runOnce(() -> stop(), this);
	}

	public Command spinCommand() {
		return Commands.run(() -> spin());
	}

	public Command reverseSpinCommand() {
		return Commands.run(() -> reverseSpin());
	}

	public Command manualControlCommand() {
		return Commands.run(() -> {
			io.setSpinVoltage(12 * (Math.pow(RobotContainer.operatorController.getLeftX(), 3)));
		});
	}
}
