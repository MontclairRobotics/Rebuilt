package frc.robot.subsystems.shooter2.hood2;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotation;
import static edu.wpi.first.units.Units.Rotations;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.constants.HoodConstants;

import static frc.robot.constants.HoodConstants.kG;
import static frc.robot.constants.HoodConstants.kS;
import static frc.robot.constants.HoodConstants.kV;

import frc.robot.util.PoseUtils;
import frc.robot.util.tunables.Tunable;

public class Hood extends SubsystemBase {

	private final HoodIO io;
	private final HoodIOInputsAutoLogged inputs = new HoodIOInputsAutoLogged();

	public PIDController pidController;
	public ArmFeedforward feedforward;

	private HoodVisualization visualization;

	public Hood(HoodIO hoodIO) {
		this.io = hoodIO;
		this.visualization = new HoodVisualization();

		Tunable kPTunable = new Tunable("Hood/Hood kP", HoodConstants.kP, (value) -> pidController.setP(value));
		Tunable kITunable = new Tunable("Hood/Hood kI", HoodConstants.kI, (value) -> pidController.setI(value));
		Tunable kDTunable = new Tunable("Hood/Hood kD", HoodConstants.kD, (value) -> pidController.setD(value));

		Tunable kSTunable = new Tunable("Hood/Hood kS", kS, (value) -> feedforward.setKs(value));
		Tunable kGTunable = new Tunable("Hood/Hood kG", kG, (value) -> feedforward.setKg(value));
		Tunable kVTunable = new Tunable("Hood/Hood kV", kV, (value) -> feedforward.setKv(value));

		pidController = new PIDController(HoodConstants.kP, HoodConstants.kI, HoodConstants.kD);
		pidController.setTolerance(HoodConstants.TOLERANCE.in(Rotations));
		feedforward = new ArmFeedforward(HoodConstants.kS, HoodConstants.kG, HoodConstants.kV);
	}

	public Angle getAngle() {
		return io.getAngle();
	}

	public Angle getAngleToPoint(Translation2d point, double heightMeters) {
		Translation2d location = PoseUtils.flipTranslationAlliance(point);
		double distance = location.minus(RobotContainer.drivetrain.getRobotPose().getTranslation()).getNorm();
		return Radians.of(Math.PI/2).minus(Radians.of(Math.atan(heightMeters/distance)));
	}

	public void applyJoystickInput() {
		double voltage = Math.pow(MathUtil.applyDeadband(RobotContainer.driverController.getLeftY(), 0.04), 3) * 3;
		double ffVoltage = feedforward.calculate(getAngle().in(Radians), 0);
		Logger.recordOutput("Hood/Feedforward Voltage", ffVoltage);
		io.setVoltage(voltage + ffVoltage);
	}

	public void setAngle(Supplier<Angle> angleSupplier) {
		setAngle(angleSupplier.get());
	}

	public void setAngle(Angle angle) {
		Logger.recordOutput("Hood/Target Angle", angle.in(Rotations));
		double pidVoltage = pidController.calculate(io.getAngle().in(Rotations), angle.in(Rotations));
		double feedforwardVoltage = feedforward.calculate(angle.in(Radians), 0);
		io.setVoltage(pidVoltage + feedforwardVoltage);
	}

	public boolean atSetpoint() {
		return pidController.atSetpoint();
	}

	public void periodic() {
		io.updateInputs(inputs);
		Logger.processInputs("Hood", inputs);
		Logger.recordOutput("Hood/Hood Angle", getAngle().in(Rotation));
		visualization.update();
		visualization.log();
	}

	public Command stopCommand() {
		return Commands.runOnce(() -> io.stop());
	}

	public Command setAngleCommand(Supplier<Angle> angleSupplier) {
		return Commands.run(() -> setAngle(angleSupplier), this);
	}

	public Command setAngleCommand(Angle angle) {
		return Commands.run(() -> setAngle(angle), this).until(() -> atSetpoint());
	}

	public Command joystickCommand() {
		return Commands.run(() -> applyJoystickInput(), this);
	}
}
