package frc.robot.subsystems.turret;

import com.ctre.phoenix6.BaseStatusSignal;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.turret.TurretIO.TurretInputs;

import org.littletonrobotics.junction.Logger;

import java.util.List;
import java.util.function.DoubleSupplier;

/**
 * The TurretIOHardware class interfaces with the TalonFX motor controller and
 * CANCoders
 * to manage turret movement and sensor readings.
 */
public class TurretSubsystem extends SubsystemBase {


    private final TurretIO io;
    private double turretPositionSetpointRadiansFromCenter = 0.0;
    private double lastModeChange = 0.0;
    private TurretInputs inputs;

    // Constructor
    public TurretSubsystem(final TurretIO io) {
        this.io = io;
        inputs = new TurretInputs();
    }
    
    public List<BaseStatusSignal> getStatusSignals() {
        return io.getStatusSignals();
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
    }

    private void setOpenLoopDutyCycleImpl(double dutyCycle) {
        Logger.recordOutput("Turret/API/setOpenLoopDutyCycle/dutyCycle", dutyCycle);
        io.setOpenLoopDutyCycle(dutyCycle);
    }

    public void setPositionSetpointImpl(double radiansFromCenter, double radPerS) {
        Logger.recordOutput("Turret/API/setPositionSetpoint/radiansFromCenter", radiansFromCenter);
        io.setPositionSetpoint(radiansFromCenter, radPerS);
    }

    // Public API
    public Command setOpenLoopDutyCycle(DoubleSupplier dutyCycle) {
        return startEnd(() -> {
            setOpenLoopDutyCycleImpl(dutyCycle.getAsDouble());
        }, () -> {
        }).withName("Turret DutyCycleControl");
    }

    // private double adjustSetpointForWrap(double radiansFromCenter) {
    //     // We have two options the raw radiansFromCenter or +/- 2 * PI.
    //     double alternative = radiansFromCenter - 2.0 * Math.PI;
    //     if (radiansFromCenter < 0.0) {
    //         alternative = radiansFromCenter + 2.0 * Math.PI;
    //     }
    //     if (Math.abs(getCurrentPosition() - alternative) < Math.abs(getCurrentPosition() - radiansFromCenter)) {
    //         return alternative;
    //     }
    //     return radiansFromCenter;
    // }

    // private boolean unwrapped(double setpoint) {
    //     // Radians comparison intentional because this is the raw value going into
    //     // rotor.
    //     return (RobotTime.getTimestampSeconds() - this.lastModeChange > 0.5) ||
    //             Util.epsilonEquals(setpoint,
    //                     getCurrentPosition(), Math.toRadians(10.0));
    // }

    // private Command positionSetpointUntilUnwrapped(DoubleSupplier radiansFromCenter, DoubleSupplier ffVel) {
    //     return run(() -> {
    //         // Intentional do not wrap turret
    //         double setpoint = radiansFromCenter.getAsDouble();
    //         setPositionSetpointImpl(setpoint, unwrapped(setpoint) ? ffVel.getAsDouble() : 0.0);
    //         turretPositionSetpointRadiansFromCenter = setpoint;
    //     }).until(() -> unwrapped(radiansFromCenter.getAsDouble()));
    // }

    // FF is in rad/s.
    // public Command positionSetpointCommand(DoubleSupplier radiansFromCenter,
    //         DoubleSupplier ffVel) {
    //     return positionSetpointUntilUnwrapped(radiansFromCenter, ffVel).andThen(
    //             run(() -> {
    //                 double setpoint = adjustSetpointForWrap(radiansFromCenter.getAsDouble());
    //                 setPositionSetpointImpl(setpoint, ffVel.getAsDouble());
    //                 turretPositionSetpointRadiansFromCenter = setpoint;
    //             })).withName("Turret positionSetpointCommand");
    // }

    // public Command waitForPosition(DoubleSupplier radiansFromCenter, double toleranceRadians) {
    //     return new WaitUntilCommand(() -> {
    //         return Math.abs(new Rotation2d(getCurrentPosition()).rotateBy(
    //                 new Rotation2d(radiansFromCenter.getAsDouble()).unaryMinus()).getRadians()) < toleranceRadians;
    //     }).withName("Turret wait for position");
    // }

    public double getSetpoint() {
        return this.turretPositionSetpointRadiansFromCenter;
    }

    // public double getCurrentPosition() {
    //     return robotState.getLatestTurretPositionRadians();
    // }

    public void setTeleopDefaultCommand() {
        this.setDefaultCommand(run(() -> {
            setPositionSetpointImpl(turretPositionSetpointRadiansFromCenter, 0.0);
        }).withName("Turret Maintain Setpoint (default)"));
    }
}
