package frc.robot.subsystems.shooter.hood;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;
import static frc.robot.constants.HoodConstants.*;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.shooter.aiming.AimingConstants;
import frc.robot.util.PoseUtils;
import frc.robot.util.tunables.LoggedTunableNumber;

public class Hood extends SubsystemBase {

    private final HoodIO io;
    private final HoodIOInputsAutoLogged inputs = new HoodIOInputsAutoLogged();
    private final HoodVisualization visualization = new HoodVisualization();

	private static final TimeInterpolatableBuffer<Angle> setpointBuffer = TimeInterpolatableBuffer.createBuffer(Hood::interpolate, AimingConstants.LATENCY * 2);

    private ArmFeedforward feedforward;

	// private final LoggedTunableNumber tunableKP = new LoggedTunableNumber("Hood/kP", SLOT0_CONFIGS.kP);
    // private final LoggedTunableNumber tunableKD = new LoggedTunableNumber("Hood/kD", SLOT0_CONFIGS.kD);
    // private final LoggedTunableNumber tunableKS = new LoggedTunableNumber("Hood/kS", SLOT0_CONFIGS.kS);
    // private final LoggedTunableNumber tunableKG = new LoggedTunableNumber("Hood/kG", SLOT0_CONFIGS.kG);

	// private final LoggedTunableNumber tunableMotionMagicCruiseVelocity = new LoggedTunableNumber("Hood/Motion Magic Cruise Velocity", MOTION_MAGIC_CONFIGS.MotionMagicCruiseVelocity);
	// private final LoggedTunableNumber tunableMotionMagicAcceleration = new LoggedTunableNumber("Hood/Motion Magic Acceleration", MOTION_MAGIC_CONFIGS.MotionMagicAcceleration);
	// private final LoggedTunableNumber tunableMotionMagicJerk = new LoggedTunableNumber("Hood/Motion Magic Jerk", MOTION_MAGIC_CONFIGS.MotionMagicJerk);

	public final LoggedTunableNumber tunableHoodAngle = new LoggedTunableNumber("Hood/Tunable Hood Angle", 0);

	private int logCounter;
	private final int loopsPerLog;

    public Hood(HoodIO io) {
        this.io = io;
        feedforward = new ArmFeedforward(kS, kG, 0);
		loopsPerLog = RobotContainer.HOOD_DEBUG ? 1 : 5;
    }

	public void periodic() {
		logCounter++;

		io.updateInputs(inputs); // need to update inputs every frame

		// Logger.recordOutput("Hood/At Time Adjusted Setpoint", atTimeAdjustedSetpoint());
		// Logger.recordOutput("Hood/Time Adjusted Setpoint", getSetpointForTime(Timer.getFPGATimestamp()));

		if(logCounter % loopsPerLog == 0) {
			Logger.processInputs("Hood", inputs);
		}

		// visualization.update();
		// visualization.log();

		// if(RobotContainer.HOOD_DEBUG || RobotBase.isSimulation()) {
		// 	updateTunables();
		// }
	}

	public static Angle interpolate(Angle startValue, Angle endValue, double t) {
        return Rotations.of(
            MathUtil.interpolate(startValue.in(Rotations), endValue.in(Rotations), t)
        );
    }

	public boolean atTimeAdjustedSetpoint() {
        return io.isAtTimeAdjustedSetpoint();
    }

    public static void recordSetpoint(Angle setpoint, double timeSecondsForSetpoint) {
        setpointBuffer.addSample(timeSecondsForSetpoint, setpoint);
    }

    public static Angle getSetpointForTime(double timeSeconds) {
        return setpointBuffer.getSample(timeSeconds).orElseGet(() -> Rotations.zero());
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
		double voltage = -Math.pow(MathUtil.applyDeadband(RobotContainer.driverController.getLeftY(), 0.04), 3) * RobotController.getBatteryVoltage();
		double ffVoltage = feedforward.calculate(getAngle().in(Radians), 0);
		// Logger.recordOutput("Hood/Feedforward Voltage", ffVoltage);
		io.setVoltage(voltage + ffVoltage);
	}

    public void setAngle(Supplier<Angle> angleSupplier, DoubleSupplier timeSecondsForSetpoint) {
		io.setAngle(angleSupplier.get(), timeSecondsForSetpoint.getAsDouble());
	}

    public void setAngle(Angle angle, DoubleSupplier timeSecondsForSetpoint) {
		io.setAngle(angle, timeSecondsForSetpoint.getAsDouble());
	}

    public boolean atSetpoint() {
		return io.isAtSetpoint();
	}

    public void updateTunables() {
		// if(tunableKP.hasChanged(hashCode())
        //         || tunableKD.hasChanged(hashCode())
        //         || tunableKS.hasChanged(hashCode())
        //         || tunableKG.hasChanged(hashCode())) {
        //     io.setGains(tunableKP.get(), tunableKD.get(), tunableKS.get(), tunableKG.get());
        // }

		// if(tunableMotionMagicAcceleration.hasChanged(hashCode())
		// 		|| tunableMotionMagicCruiseVelocity.hasChanged(hashCode())
		// 		|| tunableMotionMagicJerk.hasChanged(hashCode())) {
		// 	io.setMotionMagic(
		// 		tunableMotionMagicCruiseVelocity.get(),
		// 		tunableMotionMagicAcceleration.get(),
		// 		tunableMotionMagicJerk.get()
		// 	);
		// }
    }

	public void setNeutralMode(NeutralModeValue value) {
		io.setNeutralMode(value);
	}

    public Command stopCommand() {
		return Commands.runOnce(() -> io.stop());
	}

	public Command setVoltageCommand(double voltage) {
		return Commands.run(() -> io.setVoltage(voltage), this);
	}

	public Command setAngleCommand(Supplier<Angle> angleSupplier, DoubleSupplier timeSecondsForSetpoint) {
		return Commands.run(() -> setAngle(angleSupplier, timeSecondsForSetpoint), this);
	}

	public Command setAngleCommand(Angle angle, DoubleSupplier timeSecondsForSetpoint) {
		return Commands.run(() -> setAngle(angle, timeSecondsForSetpoint), this).until(() -> atSetpoint());
	}

	public Command joystickControlCommand() {
		return Commands.run(() -> applyJoystickInput(), this);
	}

}
