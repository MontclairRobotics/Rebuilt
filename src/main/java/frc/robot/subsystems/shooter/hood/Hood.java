// package frc.robot.subsystems.shooter.hood;

// import java.util.function.Supplier;

// import org.littletonrobotics.junction.Logger;

// import edu.wpi.first.math.MathUtil;
// import edu.wpi.first.math.controller.ArmFeedforward;
// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.geometry.Translation2d;
// import static edu.wpi.first.units.Units.Degrees;
// import static edu.wpi.first.units.Units.Radians;
// import static edu.wpi.first.units.Units.Rotations;
// import edu.wpi.first.units.measure.Angle;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.Commands;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.RobotContainer;
// import static frc.robot.constants.HoodConstants.TOLERANCE;
// import static frc.robot.constants.HoodConstants.kD;
// import static frc.robot.constants.HoodConstants.kG;
// import static frc.robot.constants.HoodConstants.kI;
// import static frc.robot.constants.HoodConstants.kP;
// import static frc.robot.constants.HoodConstants.kS;
// import static frc.robot.constants.HoodConstants.kV;
// import frc.robot.util.FieldConstants;
// import frc.robot.util.PoseUtils;
// import frc.robot.util.tunables.Tunable;

// public class Hood extends SubsystemBase {
// 	public HoodIO io;
// 	private HoodIOInputsAutoLogged inputs = new HoodIOInputsAutoLogged();

// 	public PIDController pidController;
// 	public ArmFeedforward feedforward;

// 	private HoodVisualization visualization;

// 	public Hood(HoodIO hoodIO) {
// 		this.io = hoodIO;
// 		this.visualization = new HoodVisualization();

// 		Tunable kPTunable = new Tunable("hood kP", kP, (value) -> pidController.setP(value));
// 		Tunable kITunable = new Tunable("hood kI", kI, (value) -> pidController.setI(value));
// 		Tunable kDTunable = new Tunable("hood kD", kD, (value) -> pidController.setD(value));
// 		Tunable kSTunable = new Tunable("hood ks", kS, (value) -> feedforward.setKs(value));
// 		Tunable kGTunable = new Tunable("hood kg", kG, (value) -> feedforward.setKg(value));
// 		Tunable kVTunable = new Tunable("hood kv", kV, (value) -> feedforward.setKv(value));

// 		pidController = new PIDController(kP, kI, kD);
// 		pidController.setTolerance(TOLERANCE.in(Rotations));
// 		feedforward = new ArmFeedforward(kS, kG, kV);
// 	}

// 	public Angle getAngleToHub() {
// 		// double heightMeters = FieldConstants.Hub.HEIGHT.in(Meters) - 0.30;
// 		double distance = FieldConstants.Hub.HUB_LOCATION.minus(RobotContainer.drivetrain.getRobotPose().getTranslation()).getNorm();
// 		// return Radians.of(Math.PI/2).minus(Radians.of(Math.atan(heightMeters/distance)));

// 		//until we get look up table, super fucky
// 		return Degrees.of((distance/3) * 36);
// 	}

// 	public Angle getAngle() {
// 		return io.getAngle();
// 	}

// 	public Angle getAngleToPoint(Translation2d point, double heightMeters) {
// 		Translation2d location = PoseUtils.flipTranslationAlliance(point);
// 		double distance = location.minus(RobotContainer.drivetrain.getRobotPose().getTranslation()).getNorm();
// 		return Radians.of(Math.PI/2).minus(Radians.of(Math.atan(heightMeters/distance)));
// 	}


// 	public void applyJoystickInput() {
// 		double voltage = Math.pow(MathUtil.applyDeadband(RobotContainer.driverController.getRightY(), 0.04), 3) * 3;
// 		io.setVoltage(voltage);
// 	}

// 	public void setAngle(Supplier<Angle> goalSupplier) {
// 		setAngle(goalSupplier.get());
// 	}

// 	public void setAngle(Angle goal) {
// 		double pidVoltage = pidController.calculate(io.getAngle().in(Rotations), goal.in(Rotations));
// 		double feedforwardVoltage = feedforward.calculate(io.getAngle().in(Rotations), 0);
// 		io.setVoltage(pidVoltage + feedforwardVoltage);
// 	}

// 	public boolean atSetpoint() {
// 		return pidController.atSetpoint();
// 	}

// 	public void periodic() {
// 		io.updateInputs(inputs);
// 		Logger.processInputs("Hood", inputs);
// 		visualization.update();
// 		visualization.log();
// 	}

// 	public Command stopCommand() {
// 		return Commands.runOnce(() -> io.stop());
// 	}

// 	public Command setAngleCommand(Supplier<Angle> supplier) {
// 		return Commands.run(() -> setAngle(supplier), this);
// 	}

// 	public Command setAngleCommand(Angle angle) {
// 		return Commands.run(() -> setAngle(angle), this).until(() -> atSetpoint());
// 	}

// 	public Command joystickCommand() {
// 		return Commands.run(() -> applyJoystickInput(), this);
// 	}
// }
