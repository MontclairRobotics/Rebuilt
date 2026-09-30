package frc.robot.subsystems.shooter.spindexer.indexer;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.*;
import static frc.robot.constants.IndexerConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.util.PhoenixUtil;

public class IndexerIOTalonFX implements IndexerIO {

	private TalonFX motor;
	private TalonFXConfiguration config;

	private final StatusSignal<AngularVelocity> velocitySignal;
    private final StatusSignal<Double> setpointVelocitySignal;
    private final StatusSignal<Voltage> appliedVoltageSignal;
    private final StatusSignal<Current> currentDrawAmpsSignal;
    private final StatusSignal<Temperature> tempCelciusSignal;

	private final VelocityTorqueCurrentFOC request = new VelocityTorqueCurrentFOC(0);
	private final TorqueCurrentFOC torqueRequest = new TorqueCurrentFOC(0);
	private final NeutralOut neutralOut = new NeutralOut();
	private final VoltageOut voltageOut = new VoltageOut(0);

	public IndexerIOTalonFX() {
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
        tempCelciusSignal = motor.getDeviceTemp();

		PhoenixUtil.registerStatusSignals(
			Hertz.of(50),
			velocitySignal,
			setpointVelocitySignal,
			appliedVoltageSignal,
			currentDrawAmpsSignal
		);

		// not necessary to run this fast
		tempCelciusSignal.setUpdateFrequency(4);

		motor.optimizeBusUtilization();
	}

	@Override
	public void updateInputs(IndexerIOInputs inputs) {

		BaseStatusSignal.refreshAll(
            velocitySignal,
			setpointVelocitySignal,
			appliedVoltageSignal,
			currentDrawAmpsSignal,
			tempCelciusSignal
        );

		inputs.motorConnected = BaseStatusSignal.isAllGood(
			velocitySignal,
			setpointVelocitySignal,
			appliedVoltageSignal,
			currentDrawAmpsSignal,
			tempCelciusSignal
		);

		inputs.velocity = velocitySignal.getValue();
		inputs.setpointVelocity = RotationsPerSecond.of(setpointVelocitySignal.getValue().doubleValue());

		inputs.appliedVoltage = appliedVoltageSignal.getValueAsDouble();
		inputs.currentDrawAmps = currentDrawAmpsSignal.getValueAsDouble();
		inputs.tempCelsius = tempCelciusSignal.getValueAsDouble();
		inputs.isAtSetpoint =
			Math.abs(velocitySignal.getValueAsDouble() - setpointVelocitySignal.getValueAsDouble()) < VELOCITY_TOLERANCE.in(RotationsPerSecond);
	}

	@Override
	public void setVelocity(AngularVelocity targetVelocity) {
		motor.setControl(request.withVelocity(targetVelocity));
	}

	@Override
	public void setVoltage(double voltage) {
		motor.setControl(voltageOut.withOutput(voltage));
	}

	@Override
	public void stop() {
		motor.setControl(neutralOut);
	}

	@Override
	public void setCurrent(double currentDrawAmps) {
		motor.setControl(torqueRequest.withOutput(currentDrawAmps));
	}

}
