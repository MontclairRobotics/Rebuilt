package frc.robot.subsystems.intake.rollers;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;

import static frc.robot.constants.RollersConstants.*;
import static edu.wpi.first.units.Units.Hertz;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.util.PhoenixUtil;

public class RollersIOTalonFX implements RollersIO {

	private TalonFX motor;
	private TalonFXConfiguration config;

	private final StatusSignal<AngularVelocity> velocitySignal;
    private final StatusSignal<Double> setpointVelocitySignal;
    private final StatusSignal<Voltage> appliedVoltageSignal;
    private final StatusSignal<Current> currentDrawAmpsSignal;
    private final StatusSignal<Temperature> tempCelciuSignal;

	private VelocityTorqueCurrentFOC request = new VelocityTorqueCurrentFOC(0);
	private final NeutralOut neutralOut = new NeutralOut();

	public RollersIOTalonFX() {
		motor = new TalonFX(CAN_ID, CAN_BUS);

		config = new TalonFXConfiguration()
			.withSlot0(SLOT0_CONFIGS)
			.withCurrentLimits(CURRENT_LIMITS_CONFIGS)
			.withMotorOutput(MOTOR_OUTPUT_CONFIGS)
			.withFeedback(FEEDBACK_CONFIGS);

		motor.getConfigurator().apply(config);

	 	velocitySignal = motor.getVelocity();
        setpointVelocitySignal = motor.getClosedLoopReference();
        appliedVoltageSignal = motor.getMotorVoltage();
        currentDrawAmpsSignal = motor.getTorqueCurrent();
        tempCelciuSignal = motor.getDeviceTemp();

		PhoenixUtil.registerStatusSignals(
			Hertz.of(50),
			velocitySignal,
			setpointVelocitySignal,
			appliedVoltageSignal,
			currentDrawAmpsSignal,
			tempCelciuSignal
		);

		motor.optimizeBusUtilization();
	}

	@Override
	public void updateInputs(RollersIOInputs inputs) {

		BaseStatusSignal.refreshAll(
            velocitySignal,
			setpointVelocitySignal,
			appliedVoltageSignal,
			currentDrawAmpsSignal,
			tempCelciuSignal
        );

		inputs.motorConnected = BaseStatusSignal.isAllGood(
			velocitySignal,
			setpointVelocitySignal,
			appliedVoltageSignal,
			currentDrawAmpsSignal,
			tempCelciuSignal
		);

		inputs.velocity = velocitySignal.getValue();
		inputs.setpointVelocity = RotationsPerSecond.of(setpointVelocitySignal.getValue().doubleValue());

		inputs.appliedVoltage = appliedVoltageSignal.getValueAsDouble();
		inputs.currentDrawAmps = currentDrawAmpsSignal.getValueAsDouble();
		inputs.tempCelsius = tempCelciuSignal.getValueAsDouble();
		inputs.isAtSetpoint = isAtSetpoint();
	}

	@Override
	public void setVelocity(AngularVelocity targetVelocity) {
		motor.setControl(request.withVelocity(targetVelocity));
	}

	@Override
	public void setVoltage(double voltage) {
		motor.setVoltage(voltage);
	}

	@Override
	public void stop() {
		motor.setControl(neutralOut);
	}

	@Override
	public boolean isAtSetpoint() {
		double error = motor.getClosedLoopError().getValueAsDouble();
        return Math.abs(error) < VELOCITY_TOLERANCE.in(RotationsPerSecond);
	}
}
