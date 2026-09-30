package frc.robot.subsystems.shooter.hood;

import static edu.wpi.first.units.Units.Radians;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.RobotBase;
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

	public final LoggedTunableNumber tunableHoodAngle = new LoggedTunableNumber("Hood/Tunable Hood Angle", 0);

	private int logCounter;
	private final int loopsPerLog;

    public Hood(HoodIO io) {
        this.io = io;
		loopsPerLog = RobotContainer.HOOD_DEBUG ? 1 : 5;
    }

	public void periodic() {
		logCounter++;

		// need to update this every frame
		io.updateInputs(inputs);
		Logger.processInputs("Hood", inputs);

		if(logCounter % loopsPerLog == 0) {
			// any expensive, derived logging here
		}

		if(RobotBase.isSimulation()) {
			visualization.update();
			visualization.log();
		}

	}

    public Angle getAngle() {
        return inputs.hoodAngle;
    }

    public Angle getAngleToPoint(Translation2d point, double heightMeters) {
		Translation2d location = PoseUtils.flipTranslationAlliance(point);
		double distance = location.minus(RobotContainer.drivetrain.getRobotPose().getTranslation()).getNorm();
		return Radians.of(Math.PI/2).minus(Radians.of(Math.atan(heightMeters/distance)));
	}

    public void setAngle(Supplier<Angle> angleSupplier) {
		io.setAngle(angleSupplier.get());
	}

    public void setAngle(Angle angle) {
		io.setAngle(angle);
	}

	public void stop() {
		io.stop();
	}

    public boolean atSetpoint() {
		return inputs.isAtSetpoint;
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

	public Command setAngleCommand(Supplier<Angle> angleSupplier) {
		return Commands.run(() -> setAngle(angleSupplier), this);
	}

	public Command setAngleCommand(Angle angle) {
		return Commands.run(() -> setAngle(angle), this).until(() -> atSetpoint());
	}

}
