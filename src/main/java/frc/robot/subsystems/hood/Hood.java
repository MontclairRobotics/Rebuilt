package frc.robot.subsystems.hood;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Hood extends SubsystemBase {
	public HoodIO io;
	private HoodIOInputsAutoLogged inputs = new HoodIOInputsAutoLogged();

	private HoodVisualization visualization;

	public Hood(HoodIO hoodIO) {
		this.io = hoodIO;
		this.visualization = new HoodVisualization();
	}

	public void periodic() {
		io.updateInputs(inputs);
		Logger.processInputs("Hood", inputs);
		visualization.update();
		visualization.log();
	}

	public void applyJoystickInput() {
		double voltage = Math.pow(MathUtil.applyDeadband(RobotContainer.operatorController.getRightY(), 0.04), 3) * 12;
		io.setVoltage(voltage);
	}

	public Command setAngleCommand(DoubleSupplier supplier) {
		return Commands.run(() -> io.setAngle(supplier), this);
	}

	public Command setAngleCommand(double angle) {
		return Commands.run(() -> io.setAngle(angle), this).until(() -> io.atSetpoint());
	}

	public Command joystickCommand() {
		return Commands.run(this::applyJoystickInput, this);
	}
}
