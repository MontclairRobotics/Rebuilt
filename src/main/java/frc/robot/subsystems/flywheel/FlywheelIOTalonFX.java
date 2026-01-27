package frc.robot.subsystems.flywheel;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import frc.robot.constants.FlywheelConstants;
import java.util.function.DoubleSupplier;

public class FlywheelIOTalonFX implements FlywheelIO {

	public static TalonFX motor;
	private PIDController pidController;
	private SimpleMotorFeedforward motorFeedForward;

	public FlywheelIOTalonFX() {
		motor = new TalonFX(FlywheelConstants.CAN_ID);
		pidController =
			new PIDController(
				FlywheelConstants.SLOT0_CONFIGS.kP,
				FlywheelConstants.SLOT0_CONFIGS.kI,
				FlywheelConstants.SLOT0_CONFIGS.kD
			);
		motorFeedForward =
			new SimpleMotorFeedforward(
				FlywheelConstants.SLOT0_CONFIGS.kS, FlywheelConstants.SLOT0_CONFIGS.kV
			);
	}

	public void updateInputs(FlywheelIOInputs inputs) {
		inputs.appliedVoltage = motor.getMotorVoltage().getValueAsDouble();
		inputs.tempCelcius = motor.getDeviceTemp().getValueAsDouble(); // celsius
		inputs.velocity = this.getMotorVelocity(); // RPS
		inputs.velocitySetpoint = pidController.getSetpoint();
	}

	public void stop() {
		motor.stopMotor();
	}

	public void setVoltage(double voltage) {
		motor.setVoltage(voltage);
	}

	public void setVelocityRPS(double targetVelocity) {
		double pidOutput = pidController.calculate(getMotorVelocity(), targetVelocity);
		double ffVolts = motorFeedForward.calculate(targetVelocity);
		double totalOutput = pidOutput + ffVolts;
		setVoltage(MathUtil.clamp(totalOutput, 12.0, -12.0));
	}

	public boolean atSetPoint() {
		return pidController.atSetpoint();
	}

	public double getMotorVelocity() {
		return motor.getVelocity().getValueAsDouble() / FlywheelConstants.GEARING;
	}

	public void setVelocityRPS(DoubleSupplier targetVelocitySupplier) {
		setVelocityRPS(targetVelocitySupplier.getAsDouble());
	}
}
