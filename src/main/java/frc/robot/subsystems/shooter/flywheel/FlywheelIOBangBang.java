package frc.robot.subsystems.shooter.flywheel;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.util.PhoenixUtil;

import static frc.robot.constants.FlywheelConstants.*;


public class FlywheelIOBangBang implements FlywheelIO {

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
    private final StatusSignal<Temperature> tempCelciusSignal;

    private VelocityDutyCycle dutyCycleRequest = new VelocityDutyCycle(0);
    private VelocityTorqueCurrentFOC torqueRequest = new VelocityTorqueCurrentFOC(0);
    private NeutralOut neutralOut = new NeutralOut();

    // private BangBangController bangBangController = new BangBangController();

    // when the velocity drops this much in the idle phase, we say that a ball has been shot
    private static final AngularVelocity SHOT_VELOCITY_DROP = RotationsPerSecond.of(3);

    // the velocity tolerance around the setpoint needed in order to exit the recovery phase
    private static final AngularVelocity RECOVERY_VELOCITY_TOLERANCE = VELOCITY_TOLERANCE; // may want to change these? prob keep the same

    public static enum Phase {
        SPINUP,
        IDLE,
    }

    static Phase phase = Phase.SPINUP;

    public FlywheelIOBangBang() {
        leftMotor = new TalonFX(LEFT_CAN_ID);
        rightMotor = new TalonFX(RIGHT_CAN_ID);
        rightMotor.setControl(new Follower(LEFT_CAN_ID, MotorAlignmentValue.Opposed));

        leftMotorConfig = new TalonFXConfiguration()
            .withSlot0(SLOT0_CONFIGS)
            .withMotorOutput(LEFT_MOTOR_OUTPUT_CONFIGS)
            .withTorqueCurrent(TORQUE_CURRENT_CONFIGS)
            .withFeedback(FEEDBACK_CONFIGS);

        rightMotorConfig = new TalonFXConfiguration();

        leftMotor.getConfigurator().apply(leftMotorConfig);
        rightMotor.getConfigurator().apply(rightMotorConfig);

        velocitySignal = leftMotor.getVelocity();
        accelerationSignal = leftMotor.getAcceleration();
        setpointVelocitySignal = leftMotor.getClosedLoopReference();
        setpointAccelerationSignal = leftMotor.getClosedLoopReferenceSlope();
        appliedVoltageSignal = leftMotor.getMotorVoltage();
        currentDrawAmpsSignal = leftMotor.getTorqueCurrent();
        tempCelciusSignal = leftMotor.getDeviceTemp();

        PhoenixUtil.registerStatusSignals(
            Hertz.of(50),
            velocitySignal,
            accelerationSignal,
            setpointVelocitySignal,
            setpointAccelerationSignal,
            appliedVoltageSignal,
            currentDrawAmpsSignal,
            tempCelciusSignal
        );

        leftMotor.optimizeBusUtilization();
        rightMotor.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(FlywheelIOInputs inputs) {

        BaseStatusSignal.refreshAll(
            velocitySignal,
            accelerationSignal,
            setpointVelocitySignal,
            setpointAccelerationSignal,
            appliedVoltageSignal,
            currentDrawAmpsSignal,
            tempCelciusSignal
        );

        inputs.leftMotorConnected = BaseStatusSignal.isAllGood(
            velocitySignal,
            accelerationSignal,
            setpointVelocitySignal,
            setpointAccelerationSignal,
            appliedVoltageSignal,
            currentDrawAmpsSignal,
            tempCelciusSignal
        );

        inputs.rightMotorConnected = inputs.leftMotorConnected;

        inputs.velocity = velocitySignal.getValue();
        inputs.acceleration = accelerationSignal.getValue();
        inputs.setpointVelocity = RotationsPerSecond.of(setpointVelocitySignal.getValue());
        inputs.setpointAcceleration = RotationsPerSecondPerSecond.of(setpointAccelerationSignal.getValue());

        inputs.appliedVoltage = appliedVoltageSignal.getValueAsDouble();
        inputs.currentDrawAmps = currentDrawAmpsSignal.getValueAsDouble();
        inputs.tempCelsius = tempCelciusSignal.getValueAsDouble();
        inputs.isAtSetpoint = isAtSetpoint();
    }

    @Override
    public void setVelocity(AngularVelocity targetVelocity, double timeSecondsForSetpoint) {

        double currentRPS = velocitySignal.getValue().in(RotationsPerSecond);
        double targetRPS = targetVelocity.in(RotationsPerSecond);
        double error = targetRPS - currentRPS;

        Flywheel.recordSetpoint(targetVelocity, timeSecondsForSetpoint);

        switch (phase) {

            case SPINUP:
                // when we're within tolerance, switch to idle mode
                if (Math.abs(error) < RECOVERY_VELOCITY_TOLERANCE.in(RotationsPerSecond)) {
                    phase = Phase.IDLE;
                    break;
                }
                leftMotor.setControl(dutyCycleRequest.withVelocity(targetVelocity));
                break;

            case IDLE:
                if (error > SHOT_VELOCITY_DROP.in(RotationsPerSecond)) {
                    phase = Phase.SPINUP; // we have shot a ball and need to spin up again
                    break;
                }
                leftMotor.setControl(torqueRequest.withVelocity(targetVelocity));
                break;
        }
    }

    @Override
    public void setVoltage(double voltage) {
        leftMotor.setVoltage(voltage);
    }

    @Override
    public void stop() {
        leftMotor.setControl(neutralOut);
        phase = Phase.SPINUP; // enter startup mode after we stop the flywheel
    }

    @Override
    public boolean isAtSetpoint() {
        double error = leftMotor.getClosedLoopError().getValueAsDouble();
        return Math.abs(error) < VELOCITY_TOLERANCE.in(RotationsPerSecond);
    }

    @Override
    public void setGains(double kP, double kD, double kS, double kV) {
        leftMotorConfig.Slot0.kP = kP; // bang bang only uses kP
        // leftMotorConfig.Slot0.kD = kD;
        // leftMotorConfig.Slot0.kV = kV;
        // leftMotorConfig.Slot0.kS = kS;

        leftMotor.getConfigurator().apply(leftMotorConfig.Slot0);
    }
}
