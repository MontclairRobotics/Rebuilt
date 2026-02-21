package frc.robot.subsystems.shooter.hood;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static frc.robot.constants.HoodConstants.SLOT0_CONFIGS;
import static frc.robot.constants.HoodConstants.MAX_VELOCITY_AT_SETPOINT;
import static frc.robot.constants.HoodConstants.MOTION_MAGIC_CONFIGS;
import static frc.robot.constants.HoodConstants.kG;
import static frc.robot.constants.HoodConstants.kS;

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
import frc.robot.util.tunables.LoggedTunableNumber;

public class Hood extends SubsystemBase {

    private final HoodIO io;
    private final HoodIOInputsAutoLogged inputs = new HoodIOInputsAutoLogged();
    private final HoodVisualization visualization = new HoodVisualization();

    private ArmFeedforward feedforward;

	private final LoggedTunableNumber tunableKP = new LoggedTunableNumber("Hood/kP", SLOT0_CONFIGS.kP);
    private final LoggedTunableNumber tunableKD = new LoggedTunableNumber("Hood/kD", SLOT0_CONFIGS.kD);
    private final LoggedTunableNumber tunableKS = new LoggedTunableNumber("Hood/kS", SLOT0_CONFIGS.kS);
    private final LoggedTunableNumber tunableKG = new LoggedTunableNumber("Hood/kG", SLOT0_CONFIGS.kG);

	private final LoggedTunableNumber tunableMotionMagicCruiseVelocity = new LoggedTunableNumber("Hood/Motion Magic Cruise Velocity", MOTION_MAGIC_CONFIGS.MotionMagicCruiseVelocity);
	private final LoggedTunableNumber tunableMotionMagicAcceleration = new LoggedTunableNumber("Hood/Motion Magic Acceleration", MOTION_MAGIC_CONFIGS.MotionMagicAcceleration);
	private final LoggedTunableNumber tunableMotionMagicJerk = new LoggedTunableNumber("Hood/Motion Magic Jerk", MOTION_MAGIC_CONFIGS.MotionMagicJerk);
	private final LoggedTunableNumber tunableMaxVelocityAtSetpoint = new LoggedTunableNumber("Hood/Max Velocity At Setpoint", MAX_VELOCITY_AT_SETPOINT.in(RotationsPerSecond));

	public final LoggedTunableNumber tunableHoodAngle = new LoggedTunableNumber("Hood/Tunable Hood Angle", 0);

    public Hood(HoodIO io) {
        this.io = io;
        feedforward = new ArmFeedforward(kS, kG, 0);
    }

	@Override
	public void periodic() {
		// io.updateInputs(new HoodIOInputsAutoLogged());
		Logger.processInputs("Hood", inputs);
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

    public void updateTunables() {
		if(tunableKP.hasChanged(hashCode())
                || tunableKD.hasChanged(hashCode())
                || tunableKS.hasChanged(hashCode())
                || tunableKG.hasChanged(hashCode())) {
            io.setGains(tunableKP.get(), tunableKD.get(), tunableKS.get(), tunableKG.get());
        }

		if(tunableMotionMagicAcceleration.hasChanged(hashCode())
				|| tunableMotionMagicCruiseVelocity.hasChanged(hashCode())
				|| tunableMotionMagicJerk.hasChanged(hashCode())) {
			io.setMotionMagic(
				tunableMotionMagicCruiseVelocity.get(),
				tunableMotionMagicAcceleration.get(),
				tunableMotionMagicJerk.get()
			);
		}

		if(tunableMaxVelocityAtSetpoint.hasChanged(hashCode())) MAX_VELOCITY_AT_SETPOINT = RotationsPerSecond.of(tunableMaxVelocityAtSetpoint.get());
    }

	public void setNeutralMode(NeutralModeValue value) {
		io.setNeutralMode(value);
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
