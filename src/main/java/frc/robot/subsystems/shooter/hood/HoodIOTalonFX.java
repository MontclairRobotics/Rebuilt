package frc.robot.subsystems.shooter.hood;

import static edu.wpi.first.units.Units.Hertz;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static frc.robot.constants.HoodConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
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

    private final MotionMagicVoltage request = new MotionMagicVoltage(0).withEnableFOC(true);
    private final NeutralOut neutralOut = new NeutralOut();

    public HoodIOTalonFX() {
        motor = new TalonFX(CAN_ID, CAN_BUS);
        encoder = new CANcoder(ENCODER_ID, CAN_BUS);

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

        motor.getConfigurator().apply(config);
        encoder.getConfigurator().apply(ENCODER_CONFIGS);

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
            currentDrawAmpsSignal,
            tempCelsiusSignal
        );

        motor.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(HoodIOInputs inputs) {
        inputs.motorConnected = BaseStatusSignal.isAllGood(
            positionSignal,
            setpointPositionSignal,
            velocitySignal,
            appliedVoltageSignal,
            currentDrawAmpsSignal,
            tempCelsiusSignal
        );

        inputs.hoodAngle = motor.getPosition().getValue();

        // inputs.appliedVoltage = appliedVoltageSignal.getValueAsDouble();
        // inputs.currentDrawAmps = currentDrawAmpsSignal.getValueAsDouble();
        // inputs.tempCelcius = tempCelsiusSignal.getValueAsDouble();

        // inputs.hoodAngle = positionSignal.getValue();
        // inputs.hoodAngleSetpoint = Rotations.of(setpointPositionSignal.getValue());
        // inputs.hoodVelocity = velocitySignal.getValue();

        // inputs.isAtSetpoint = isAtSetpoint();
    }

    @Override
    public void setAngle(Angle angle) {
        motor.setControl(request.withPosition(angle));
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
        return Math.abs(error) < TOLERANCE.in(Rotations)
            && Math.abs(velocitySignal.getValueAsDouble()) < MAX_VELOCITY_AT_SETPOINT.in(RotationsPerSecond);
    }

    @Override
    public void setGains(double kP, double kD, double kS, double kG) {
        config.Slot0.kP = kP;
        config.Slot0.kD = kD;
        config.Slot0.kS = kS;
        config.Slot0.kG = kG;

        motor.getConfigurator().apply(config.Slot0);
    }

    @Override
    public void setMotionMagic(double velocity, double acceleration, double jerk) {
        config.MotionMagic.MotionMagicCruiseVelocity = velocity;
        config.MotionMagic.MotionMagicAcceleration = acceleration;
        config.MotionMagic.MotionMagicJerk = jerk;

        motor.getConfigurator().apply(config.MotionMagic);
    }

    @Override
    public void setNeutralMode(NeutralModeValue value) {
        motor.setNeutralMode(value);
    }
}
