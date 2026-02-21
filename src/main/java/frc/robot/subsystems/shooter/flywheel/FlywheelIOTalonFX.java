package frc.robot.subsystems.shooter.flywheel;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.util.PhoenixUtil;

import static edu.wpi.first.units.Units.*;
import static frc.robot.constants.FlywheelConstants.CAN_BUS;
import static frc.robot.constants.FlywheelConstants.CURRENT_LIMITS_CONFIGS;
import static frc.robot.constants.FlywheelConstants.FEEDBACK_CONFIGS;
import static frc.robot.constants.FlywheelConstants.LEFT_CAN_ID;
import static frc.robot.constants.FlywheelConstants.LEFT_MOTOR_OUTPUT_CONFIGS;
import static frc.robot.constants.FlywheelConstants.RIGHT_CAN_ID;
import static frc.robot.constants.FlywheelConstants.RIGHT_MOTOR_OUTPUT_CONFIGS;
import static frc.robot.constants.FlywheelConstants.SLOT0_CONFIGS;
import static frc.robot.constants.FlywheelConstants.VELOCITY_TOLERANCE;

public class FlywheelIOTalonFX implements FlywheelIO{

    private TalonFX leftMotor;
    private TalonFX rightMotor;

    private TalonFXConfiguration leftMotorConfig;
    private TalonFXConfiguration rightMotorConfig;

    private final StatusSignal<AngularVelocity> velocitySignal;
    private final StatusSignal<AngularAcceleration> accelerationSignal;
    private final StatusSignal<Double> setpointVelocitySignal;
    private final StatusSignal<Double> setpointAccelerationSignal;
    private final StatusSignal<Voltage> appliedVoltageSignal;
    private final StatusSignal<Current> currentDrawAmpsSignal;
    private final StatusSignal<Temperature> tempCelciuSignal;

    private final VelocityTorqueCurrentFOC request = new VelocityTorqueCurrentFOC(0);
    private final NeutralOut neutralOut = new NeutralOut();

    public FlywheelIOTalonFX() {
        leftMotor = new TalonFX(LEFT_CAN_ID, CAN_BUS);
        rightMotor = new TalonFX(RIGHT_CAN_ID, CAN_BUS);
        rightMotor.setControl(new Follower(LEFT_CAN_ID, MotorAlignmentValue.Aligned));

        leftMotorConfig = new TalonFXConfiguration()
            .withSlot0(SLOT0_CONFIGS)
            .withCurrentLimits(CURRENT_LIMITS_CONFIGS)
            .withMotorOutput(LEFT_MOTOR_OUTPUT_CONFIGS)
            .withFeedback(FEEDBACK_CONFIGS);

        rightMotorConfig = new TalonFXConfiguration()
            .withCurrentLimits(CURRENT_LIMITS_CONFIGS)
            .withMotorOutput(RIGHT_MOTOR_OUTPUT_CONFIGS);

        leftMotor.getConfigurator().apply(leftMotorConfig);
        rightMotor.getConfigurator().apply(rightMotorConfig);

        velocitySignal = leftMotor.getVelocity();
        accelerationSignal = leftMotor.getAcceleration();
        setpointVelocitySignal = leftMotor.getClosedLoopReference();
        setpointAccelerationSignal = leftMotor.getClosedLoopReferenceSlope();
        appliedVoltageSignal = leftMotor.getMotorVoltage();
        currentDrawAmpsSignal = leftMotor.getTorqueCurrent();
        tempCelciuSignal = leftMotor.getDeviceTemp();

        PhoenixUtil.registerStatusSignals(
            Hertz.of(50),
            velocitySignal,
            accelerationSignal,
            setpointVelocitySignal,
            setpointAccelerationSignal,
            appliedVoltageSignal,
            currentDrawAmpsSignal
        );

        leftMotor.optimizeBusUtilization();
        rightMotor.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(FlywheelIOInputs inputs) {
        inputs.leftMotorConnected = BaseStatusSignal.isAllGood(
            velocitySignal,
            accelerationSignal,
            setpointVelocitySignal,
            setpointAccelerationSignal,
            appliedVoltageSignal,
            currentDrawAmpsSignal,
            tempCelciuSignal
        );

        inputs.rightMotorConnected = inputs.leftMotorConnected;

        inputs.velocity = velocitySignal.getValue();
        inputs.acceleration = accelerationSignal.getValue();
        inputs.setpointVelocity = RotationsPerSecond.of(setpointVelocitySignal.getValue());
        inputs.setpointAcceleration = RotationsPerSecondPerSecond.of(setpointAccelerationSignal.getValue());

        inputs.appliedVoltage = appliedVoltageSignal.getValueAsDouble();
        inputs.currentDrawAmps = currentDrawAmpsSignal.getValueAsDouble();
        inputs.tempCelsius = tempCelciuSignal.getValueAsDouble();
        inputs.isAtSetpoint = false;
    }

    @Override
    public void setVelocity(AngularVelocity targetVelocity) {
        leftMotor.setControl(request.withVelocity(targetVelocity));
    }

    @Override
    public void setVoltage(double voltage) {
        leftMotor.setControl(new VoltageOut(voltage));
    }

    @Override
    public void stop() {
        leftMotor.setControl(neutralOut);
    }

    @Override
    public boolean isAtSetpoint() {
        double error = leftMotor.getClosedLoopError().getValueAsDouble();
        return Math.abs(error) < VELOCITY_TOLERANCE.in(RotationsPerSecond);
    }

    @Override
    public void setGains(double kP, double kD, double kS, double kV) {
        leftMotorConfig.Slot0.kP = kP;
        leftMotorConfig.Slot0.kD = kD;
        leftMotorConfig.Slot0.kV = kV;
        leftMotorConfig.Slot0.kS = kS;

        leftMotor.getConfigurator().apply(leftMotorConfig.Slot0);
    }

}
