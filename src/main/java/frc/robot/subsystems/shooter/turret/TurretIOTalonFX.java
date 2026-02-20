package frc.robot.subsystems.shooter.turret;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.util.PhoenixUtil;

import static edu.wpi.first.units.Units.Hertz;
import static edu.wpi.first.units.Units.Rotations;
import static frc.robot.constants.TurretConstants.*;

public class TurretIOTalonFX implements TurretIO {

    private final TalonFX motor;
    private final CANcoder encoder;

    private final TalonFXConfiguration config;

    private final StatusSignal<Angle> positionSignal;
    private final StatusSignal<Double> setpointPositionSignal;
    private final StatusSignal<AngularVelocity> velocitySignal;
    private final StatusSignal<Voltage> appliedVoltageSignal;
    private final StatusSignal<Current> currentDrawAmpsSignal;
    private final StatusSignal<Temperature> tempCelsiusSignal;

    private final MotionMagicVoltage request = new MotionMagicVoltage(0);
    private final NeutralOut neutralOut = new NeutralOut();

    public TurretIOTalonFX() {
        motor = new TalonFX(CAN_ID, CAN_BUS);
        encoder = new CANcoder(ENCODER_PORT);

        config = new TalonFXConfiguration()
            .withSlot0(SLOT0_CONFIGS)
            .withCurrentLimits(CURRENT_LIMITS_CONFIGS)
            .withMotorOutput(MOTOR_OUTPUT_CONFIGS)
            .withFeedback(FEEDBACK_CONFIGS);

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
    public void updateInputs(TurretIOInputs inputs) {
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

        // these status signals are already in the MOTORS frame of reference
        inputs.motorVelocity = velocitySignal.getValue();
        inputs.motorPosition = positionSignal.getValue();

        // because positionSignal is in the MOTORS frame of reference, we divide by GEARING to convert back 
        // to the MECHANISMS frame of reference
        inputs.robotRelativeAngle = positionSignal.getValue().div(GEARING);
        inputs.fieldRelativeAngle = Turret.toFieldRelativeAngle(inputs.robotRelativeAngle);

        // setpointPositionSignal is in the MOTORS frame of reference, we divide by GEARING to convert back 
        // to the MECHANISMS frame of reference
        inputs.robotRelativeAngleSetpoint = Rotations.of(setpointPositionSignal.getValueAsDouble()).div(GEARING);
        inputs.motorPositionSetpoint = Rotations.of(setpointPositionSignal.getValueAsDouble());
    }

    @Override
    public void setRobotRelativeAngle(Angle angle) {
        // we take in a robot relative angle, but the PIDController uses MOTOR SHAFT rotations
        double targetMotorPosition = angle.times(GEARING).in(Rotations);
        motor.setControl(request.withPosition(targetMotorPosition));
    }

    @Override
    public void setVoltage(double voltage) {
        motor.setControl(new VoltageOut(voltage));
    }

    @Override
    public void stop() {
        motor.setControl(neutralOut);
    }

    @Override
    public boolean isAtSetpoint() {
        double error = motor.getClosedLoopError().getValueAsDouble();
        return Math.abs(error) < ANGLE_TOLERANCE.in(Rotations);
    }

    @Override
    public void setGains(double kP, double kD, double kS) {
        config.Slot0.kP = kP;
        config.Slot0.kD = kD;
        config.Slot0.kS = kS;

        motor.getConfigurator().apply(config.Slot0);
    }

}
