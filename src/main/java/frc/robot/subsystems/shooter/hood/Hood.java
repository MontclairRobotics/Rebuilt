package frc.robot.subsystems.shooter.hood;

import static edu.wpi.first.units.Units.Radians;
import static frc.robot.constants.HoodConstants.*;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.util.PoseUtils;

public class Hood extends SubsystemBase {

    private final HoodIO io;
    private final HoodIOInputsAutoLogged inputs = new HoodIOInputsAutoLogged();
    private final HoodVisualization visualization = new HoodVisualization();

    private ArmFeedforward feedforward;

	public double tunedAngleDegrees = 0;

	// private final Tunable kPTunable;
	// private final Tunable kDTunable;
	// private final Tunable kSTunable;
	// private final Tunable kGTunable;

	// private final Tunable tunableMotionMagicCruiseVelocity;
	// private final Tunable tunableMotionMagicAcceleration;
	// private final Tunable tunableMotionMagicJerk;

	// private final Tunable tunableMaxVelocityAtSetpoint;
	// public final Tunable tunableHoodAngle;

	public Hood(HoodIO io) {
        this.io = io;
        feedforward = new ArmFeedforward(kS, kG, 0);

		// kPTunable = new Tunable(
		// 	"Hood/Hood kP",
		// 	kP,
		// 	(value) -> {
		// 		io.setGains(value, SLOT0_CONFIGS.kD, SLOT0_CONFIGS.kS, SLOT0_CONFIGS.kG);
		// 	}
		// );

		// kDTunable = new Tunable(
		// 	"Hood/Hood kD",
		// 	kP,
		// 	(value) -> {
		// 		io.setGains(SLOT0_CONFIGS.kP, value, SLOT0_CONFIGS.kS, SLOT0_CONFIGS.kG);
		// 	}
		// );

		// kSTunable = new Tunable(
		// 	"Hood/Hood kS",
		// 	kP,
		// 	(value) -> {
		// 		io.setGains(SLOT0_CONFIGS.kP, SLOT0_CONFIGS.kD, value, SLOT0_CONFIGS.kG);
		// 	}
		// );

		// kGTunable = new Tunable(
		// 	"Hood/Hood kG",
		// 	kP,
		// 	(value) -> {
		// 		io.setGains(SLOT0_CONFIGS.kP, SLOT0_CONFIGS.kD, SLOT0_CONFIGS.kS, value);
		// 	}
		// );

		// tunableMotionMagicCruiseVelocity = new Tunable(
		// 	"Hood/Motion Magic Cruise Velocity",
		// 	MOTION_MAGIC_CONFIGS.MotionMagicCruiseVelocity,
		// 	(value) -> {
		// 		io.setMotionMagic(
		// 			value,
		// 			MOTION_MAGIC_CONFIGS.MotionMagicAcceleration,
		// 			MOTION_MAGIC_CONFIGS.MotionMagicJerk
		// 		);
		// 	}
		// );

		// tunableMotionMagicAcceleration = new Tunable(
		// 	"Hood/Motion Magic Acceleration",
		// 	MOTION_MAGIC_CONFIGS.MotionMagicAcceleration,
		// 	(value) -> {
		// 		io.setMotionMagic(
		// 			MOTION_MAGIC_CONFIGS.MotionMagicCruiseVelocity,
		// 			value,
		// 			MOTION_MAGIC_CONFIGS.MotionMagicJerk
		// 		);
		// 	}
		// );

		// tunableMotionMagicJerk = new Tunable(
		// 	"Hood/Motion Magic Jerk",
		// 	MOTION_MAGIC_CONFIGS.MotionMagicJerk,
		// 	(value) -> {
		// 		io.setMotionMagic(
		// 			MOTION_MAGIC_CONFIGS.MotionMagicCruiseVelocity,
		// 			MOTION_MAGIC_CONFIGS.MotionMagicAcceleration,
		// 			value
		// 		);
		// 	}
		// );

		// tunableMaxVelocityAtSetpoint = new Tunable(
		// 	"Hood/Max Velocity At Setpoint",
		// 	MAX_VELOCITY_AT_SETPOINT.in(RotationsPerSecond),
		// 	(value) -> MAX_VELOCITY_AT_SETPOINT = RotationsPerSecond.of(value)
		// );

		// tunableHoodAngle = new Tunable(
		// 	"Hood/Tunable Hood Angle",
		// 	0,
		// 	(value) -> tunedAngleDegrees = value
		// );
	}

	@Override
	public void periodic() {
		// io.updateInputs(inputs);
		// Logger.processInputs("Hood", inputs);
		// visualization.update();
		// visualization.log();
        // updateTunables();
	}

    public Angle getAngle() {
        return inputs.hoodAngle;
    }

    public Angle getAngleToPoint(Translation2d point, double heightMeters) {
		Translation2d location = PoseUtils.flipTranslationAlliance(point);
		double distance = location.minus(RobotContainer.drivetrain.getRobotPose().getTranslation()).getNorm();
		return Radians.of(Math.PI/2).minus(Radians.of(Math.atan(heightMeters/distance)));
	}

    public void applyJoystickInput() {
		double voltage = Math.pow(MathUtil.applyDeadband(RobotContainer.driverController.getLeftY(), 0.04), 3) * 12;
		double ffVoltage = feedforward.calculate(getAngle().in(Radians), 0);
		Logger.recordOutput("Hood/Feedforward Voltage", ffVoltage);
		io.setVoltage(voltage + ffVoltage);
	}

    public void setAngle(Supplier<Angle> angleSupplier) {
		io.setAngle(angleSupplier.get());
	}

    public void setAngle(Angle angle) {
		io.setAngle(angle);
	}

    public boolean atSetpoint() {
		return io.isAtSetpoint();
	}

    // public void updateTunables() {
	// 	if(tunableKP.hasChanged(hashCode())
    //             || tunableKD.hasChanged(hashCode())
    //             || tunableKS.hasChanged(hashCode())
    //             || tunableKG.hasChanged(hashCode())) {
    //         io.setGains(tunableKP.get(), tunableKD.get(), tunableKS.get(), tunableKG.get());
    //     }

		// if(tunableMotionMagicAcceleration.hasChanged(hashCode())
		// 		|| tunableMotionMagicCruiseVelocity.hasChanged(hashCode())
		// 		|| tunableMotionMagicJerk.hasChanged(hashCode())) {
		// 	io.setMotionMagic(
		// 		tunableMotionMagicCruiseVelocity.get(),
		// 		tunableMotionMagicAcceleration.get(),
		// 		tunableMotionMagicJerk.get()
		// 	);
		// }

	// 	if(tunableMaxVelocityAtSetpoint.hasChanged(hashCode())) MAX_VELOCITY_AT_SETPOINT = RotationsPerSecond.of(tunableMaxVelocityAtSetpoint.get());
    // }

	public void setNeutralMode(NeutralModeValue value) {
		io.setNeutralMode(value);
	}

	public Command setVoltageCommand(double voltage) {
		return Commands.run(() -> io.setVoltage(voltage));
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

	public Command joystickControlCommand() {
		return Commands.run(() -> applyJoystickInput(), this);
	}
}
