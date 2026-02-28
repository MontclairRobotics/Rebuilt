package frc.robot.subsystems.intake.pivot;

import static frc.robot.constants.PivotConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

import java.util.function.Supplier;

public class Pivot extends SubsystemBase {

	public PivotIO io;
	private PivotIOInputsAutoLogged inputs = new PivotIOInputsAutoLogged();

	private PivotVisualization visualization;

	public Pivot(PivotIO io) {
		this.io = io;
		this.visualization = new PivotVisualization();
		io.setGains(kP, kD, kS, kG);
		io.setMotionMagic(MOTION_MAGIC_CRUISE_VELOCITY, MOTION_MAGIC_ACCELERATION, MOTION_MAGIC_JERK);
	}

	public boolean atSetpoint() {
		return io.isAtSetpoint();
	}

	public Command testVoltageCommand() {
		return Commands.run(() -> io.setVoltage(2));
	}

	public void setPivotAngle(Angle angle) {
		io.setAngle(angle);
	}

	public void joystickControl() {
		double voltage = MathUtil.copyDirectionPow(MathUtil.applyDeadband(-RobotContainer.driverController.getRightY(), 0.04), 1.5) * 12;
		// Logger.recordOutput("Pivot/JoystickVoltage", voltage);
		// Logger.recordOutput("Pivot/RawAxis", RobotContainer.driverController.getRightY());
		io.setVoltage(voltage);
	}

	public Command stopCommand() {
		return Commands.runOnce(() -> io.stop());
	}

	public Command goToAngleCommand(Angle angle) {
		return Commands.run(() -> setPivotAngle(angle), this)
			.until(this::atSetpoint)
			.finallyDo(() -> {
				io.stop();
			});
	}

	public Command goToAngleCommand(Supplier<Angle> angle) {
		return Commands.run(() -> setPivotAngle(angle.get()), this)
			.finallyDo(() -> {
				io.stop();
			});
	}

	public Command joystickControlCommand() {
		return Commands.run(this::joystickControl, this);
	}

	@Override
	public void periodic() {
		// io.updateInputs(inputs);
		// Logger.processInputs("Pivot", inputs);
		// visualization.update();
		// visualization.log();
	}
}
