package frc.robot.subsystems.shooter.hood;

import static edu.wpi.first.units.Units.Radians;
import static frc.robot.constants.HoodConstants.SLOT0_CONFIGS;
import static frc.robot.constants.HoodConstants.kG;
import static frc.robot.constants.HoodConstants.kS;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

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

    public Hood(HoodIO io) {
        this.io = io;
        feedforward = new ArmFeedforward(kS, kG, 0);
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
		double voltage = Math.pow(MathUtil.applyDeadband(RobotContainer.driverController.getLeftY(), 0.04), 3) * 3;
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
  
    }

    public void periodic() {
		io.updateInputs(inputs);
		Logger.processInputs("Hood", inputs);
		visualization.update();
		visualization.log();
        updateTunables();
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
