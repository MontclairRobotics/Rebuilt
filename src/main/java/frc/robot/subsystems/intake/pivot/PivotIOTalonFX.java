package frc.robot.subsystems.intake.pivot;

import static edu.wpi.first.units.Units.Hertz;
import static edu.wpi.first.units.Units.Rotations;
import static frc.robot.constants.PivotConstants.*;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.util.PhoenixUtil;

public class PivotIOTalonFX implements PivotIO {

	private final TalonFX motor;
	private final CANcoder encoder;

	private TalonFXConfiguration configs = new TalonFXConfiguration();

    private final StatusSignal<Angle> positionSignal;
    private final StatusSignal<Double> setpointPositionSignal;
    private final StatusSignal<AngularVelocity> velocitySignal;
    private final StatusSignal<Voltage> appliedVoltageSignal;
    private final StatusSignal<Current> currentDrawAmpsSignal;
    private final StatusSignal<Temperature> tempCelsiusSignal;

    private final MotionMagicVoltage request = new MotionMagicVoltage(0).withEnableFOC(true);
    private final NeutralOut neutralOut = new NeutralOut();
    private final VoltageOut voltageOut = new VoltageOut(0);

	public PivotIOTalonFX() {
        motor = new TalonFX(CAN_ID, CAN_BUS);
        encoder = new CANcoder(ENCODER_ID, CAN_BUS);

        configs = new TalonFXConfiguration()
            .withSlot0(SLOT0_CONFIGS)
            .withCurrentLimits(CURRENT_LIMITS_CONFIGS)
            .withFeedback(FEEDBACK_CONFIGS)
            .withMotorOutput(MOTOR_OUTPUT_CONFIGS)
            .withMotionMagic(MOTION_MAGIC_CONFIGS);

        configs.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        configs.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        configs.SoftwareLimitSwitch.ForwardSoftLimitThreshold = MAX_ANGLE.in(Rotations);
        configs.SoftwareLimitSwitch.ReverseSoftLimitThreshold = MIN_ANGLE.in(Rotations);

        encoder.getConfigurator().apply(ENCODER_CONFIGS);
        motor.getConfigurator().apply(configs);
        encoder.setPosition(encoder.getAbsolutePosition().getValueAsDouble());

        positionSignal = motor.getPosition();
        setpointPositionSignal = motor.getClosedLoopReference();
        velocitySignal = motor.getVelocity();
        appliedVoltageSignal = motor.getMotorVoltage();
        currentDrawAmpsSignal = motor.getStatorCurrent();
        tempCelsiusSignal = motor.getDeviceTemp();

        PhoenixUtil.registerStatusSignals(
            Hertz.of(50),
            positionSignal,
            setpointPositionSignal,
            velocitySignal,
            appliedVoltageSignal,
            currentDrawAmpsSignal
        );

        // not necessary to run this fast
        tempCelsiusSignal.setUpdateFrequency(4);

        motor.optimizeBusUtilization();
	}

	public void updateInputs(PivotIOInputs inputs) {

		BaseStatusSignal.refreshAll(
            positionSignal,
            setpointPositionSignal,
            velocitySignal,
            appliedVoltageSignal,
            currentDrawAmpsSignal,
            tempCelsiusSignal
        );

        inputs.motorConnected = BaseStatusSignal.isAllGood(
            positionSignal,
            setpointPositionSignal,
            velocitySignal,
            appliedVoltageSignal,
            currentDrawAmpsSignal,
            tempCelsiusSignal
        );

        inputs.appliedVoltage = appliedVoltageSignal.getValueAsDouble();
        inputs.currentDrawAmps = currentDrawAmpsSignal.getValueAsDouble();
        inputs.tempCelcius = tempCelsiusSignal.getValueAsDouble();

        inputs.angle = positionSignal.getValue();
        inputs.angleSetpoint = Rotations.of(setpointPositionSignal.getValue());
        inputs.velocity = velocitySignal.getValue();

        inputs.isAtSetpoint = 
            Math.abs(positionSignal.getValueAsDouble() - setpointPositionSignal.getValueAsDouble()) < TOLERANCE.in(Rotations);
	}

	@Override
	public void setVoltage(double voltage) {
		motor.setControl(voltageOut.withOutput(voltage));
	}

	@Override
	public void setAngle(Angle angle) {
		motor.setControl(request.withPosition(angle));
	}

	@Override
	public void stop() {
		motor.setControl(neutralOut);
	}

    @Override
    public void resetEncoderPosition() {
        encoder.setPosition(encoder.getAbsolutePosition().getValueAsDouble());
    }

	public Angle getAngle() {
		if (encoder.isConnected()) {
			return encoder.getPosition().getValue();
		} else {
			return motor.getPosition().getValue();
		}
	}
	
    @Override
	public void setNeutralMode(NeutralModeValue value) {
		motor.setNeutralMode(value);
	}

}
