package frc.robot.subsystems.shooter.hood;

import static edu.wpi.first.units.Units.Hertz;
import static edu.wpi.first.units.Units.Rotations;
import static frc.robot.constants.HoodConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
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

public class HoodIOTalonFX implements HoodIO {

    private TalonFX motor;
    private CANcoder encoder;

    private final TalonFXConfiguration config;

    private final StatusSignal<Angle> positionSignal;
    private final StatusSignal<Double> setpointPositionSignal;
    private final StatusSignal<AngularVelocity> velocitySignal;
    private final StatusSignal<Voltage> appliedVoltageSignal;
    private final StatusSignal<Current> currentDrawAmpsSignal;
    private final StatusSignal<Temperature> tempCelsiusSignal;

    private final PositionVoltage request = new PositionVoltage(0).withEnableFOC(true);
    private final NeutralOut neutralOut = new NeutralOut();
    private final VoltageOut voltageOut = new VoltageOut(0);

    public HoodIOTalonFX() {
        motor = new TalonFX(CAN_ID);
        encoder = new CANcoder(ENCODER_ID);

        config = new TalonFXConfiguration()
            .withSlot0(SLOT0_CONFIGS)
            .withCurrentLimits(CURRENT_LIMITS_CONFIGS)
            .withMotorOutput(MOTOR_OUTPUT_CONFIGS)
            .withFeedback(FEEDBACK_CONFIGS)
            .withMotionMagic(MOTION_MAGIC_CONFIGS);

        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = MAX_ANGLE.in(Rotations);
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = MIN_ANGLE.in(Rotations);

        encoder.getConfigurator().apply(ENCODER_CONFIGS);
        motor.getConfigurator().apply(config);
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

    @Override
    public void updateInputs(HoodIOInputs inputs) {

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

        inputs.hoodAngle = positionSignal.getValue();
        inputs.hoodAngleSetpoint = Rotations.of(setpointPositionSignal.getValue());
        inputs.hoodVelocity = velocitySignal.getValue();

        inputs.isAtSetpoint =
            Math.abs(positionSignal.getValueAsDouble() - setpointPositionSignal.getValueAsDouble()) < TOLERANCE.in(Rotations);
    }

    @Override
    public void setAngle(Angle angle) {
        motor.setControl(request.withPosition(angle));
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
    public void resetEncoderPosition() {
        encoder.setPosition(encoder.getAbsolutePosition().getValueAsDouble());
    }

    @Override
    public void setNeutralMode(NeutralModeValue value) {
        motor.setNeutralMode(value);
    }
}
