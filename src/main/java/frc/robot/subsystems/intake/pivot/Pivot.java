package frc.robot.subsystems.intake.pivot;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;
import static frc.robot.constants.PivotConstants.TOLERANCE;
import static frc.robot.constants.PivotConstants.kD;
import static frc.robot.constants.PivotConstants.kG;
import static frc.robot.constants.PivotConstants.kI;
import static frc.robot.constants.PivotConstants.kP;
import static frc.robot.constants.PivotConstants.kS;
import static frc.robot.constants.PivotConstants.kV;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.util.tunables.Tunable;

import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class Pivot extends SubsystemBase {

	public PivotIO io;
	private PivotIOInputsAutoLogged inputs = new PivotIOInputsAutoLogged();

	private PIDController pidController;
	private ArmFeedforward feedforward;
	private PivotVisualization visualization;

	public Pivot(PivotIO io) {
		this.io = io;
		this.visualization = new PivotVisualization();

		pidController = new PIDController(kP, kI, kD);
		feedforward = new ArmFeedforward(kS, kG, kV);

		Tunable kP = new Tunable("Pivot/kP", pidController.getP(), (val) -> pidController.setP(val));
		Tunable kD = new Tunable("Pivot/kD", pidController.getD(), (val) -> pidController.setD(val));
		Tunable kG = new Tunable("Pivot/kG", feedforward.getKg(), (val) -> feedforward.setKg(val));

		pidController.disableContinuousInput();
		pidController.setTolerance(TOLERANCE.in(Rotations));
	}

	@Override
	public void periodic() {
		// io.updateInputs(inputs);
		// Logger.processInputs("Pivot", inputs);
		// visualization.update();
		// visualization.log();
	}

	public boolean atSetpoint() {
		return pidController.atSetpoint();
	}

	public Command testVoltageCommand() {
		return Commands.run(() -> io.setVoltage(2));
	}

	public void setPivotAngle(Angle angle) {
		double pidOutput = pidController.calculate(io.getAngle().in(Rotations), angle.in(Rotations));
		double ffOutput = feedforward.calculate(io.getAngle().in(Radians), 0);
		double totalOutput = MathUtil.clamp(pidOutput + ffOutput, -12, 12);
		io.setVoltage(totalOutput);
	}

	public void joystickControl() {
		double voltage = MathUtil.copyDirectionPow(MathUtil.applyDeadband(-RobotContainer.driverController.getRightY(), 0.04), 1.5) * 12;
		Logger.recordOutput("Pivot/JoystickVoltage", voltage);
		Logger.recordOutput("Pivot/RawAxis", RobotContainer.driverController.getRightY());
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
				pidController.reset();
			});
	}

	public Command goToAngleCommand(Supplier<Angle> angle) {
		return Commands.run(() -> setPivotAngle(angle.get()), this)
			.finallyDo(() -> {
				io.stop();
				pidController.reset();
			});
	}

	public Command joystickControlCommand() {
		return Commands.run(this::joystickControl, this);
	}

}
