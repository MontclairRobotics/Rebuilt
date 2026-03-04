package frc.robot.subsystems.shooter.flywheel;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static frc.robot.constants.FlywheelConstants.*;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.shooter.aiming.AimingConstants;
import frc.robot.util.tunables.LoggedTunableNumber;

public class Flywheel extends SubsystemBase {

    private final FlywheelIO io;
    private final FlywheelIOInputsAutoLogged inputs = new FlywheelIOInputsAutoLogged();

    private static final TimeInterpolatableBuffer<AngularVelocity> setpointBuffer = TimeInterpolatableBuffer.createBuffer(Flywheel::interpolate, AimingConstants.LATENCY * 2);

    private LoggedTunableNumber tunableKP;
    private LoggedTunableNumber tunableKS;
    private LoggedTunableNumber tunableKV;

    public LoggedTunableNumber tuningFlywheelSpeed;

    private int logCounter;
    private final int loopsPerLog;

    public Flywheel(FlywheelIO io) {
        this.io = io;
        loopsPerLog = RobotContainer.FLYWHEEL_DEBUG ? 1 : 5;

        if(RobotContainer.FLYWHEEL_DEBUG) {
            tunableKP = new LoggedTunableNumber("Flywheel/kP", SLOT0_CONFIGS.kP);
            tunableKS = new LoggedTunableNumber("Flywheel/kS", SLOT0_CONFIGS.kS);
            tunableKV = new LoggedTunableNumber("Flywheel/kV", SLOT0_CONFIGS.kV);

            tuningFlywheelSpeed = new LoggedTunableNumber("Flywheel/TuningFlywheelRPS", 0);
        }
    }

    public static AngularVelocity interpolate(AngularVelocity startValue, AngularVelocity endValue, double t) {
        return RotationsPerSecond.of(
            MathUtil.interpolate(startValue.in(RotationsPerSecond), endValue.in(RotationsPerSecond), t)
        );
    }

    public boolean atGoal() {
        return io.isAtSetpoint();
    }

    public static void recordSetpoint(AngularVelocity setpointVelocity, double timeSecondsForSetpoint) {
        setpointBuffer.addSample(timeSecondsForSetpoint, setpointVelocity);
    }

    public static AngularVelocity getSetpointForTime(double timeSeconds) {
        return setpointBuffer.getSample(timeSeconds).orElseGet(() -> RotationsPerSecond.zero());
    }

    private void updateTunables() {
        if(tunableKP.hasChanged(hashCode())
                || tunableKS.hasChanged(hashCode())
                || tunableKV.hasChanged(hashCode())) {
            io.setGains(tunableKP.get(), 0, tunableKS.get(), tunableKV.get());
        }
    }

    public void periodic() {
        logCounter++;

        io.updateInputs(inputs); // need to update inputs every frame

        if(logCounter % loopsPerLog == 0) {
            Logger.processInputs("Flywheel", inputs);
            Logger.recordOutput("Flywheel/Mode", FlywheelIOBangBang.phase);
        }

        if(RobotContainer.FLYWHEEL_DEBUG) updateTunables();
    }

    public void setVelocity(AngularVelocity targetVelocity, double timeSecondsForSetpoint) {
        io.setVelocity(targetVelocity, timeSecondsForSetpoint);
    }

    public void setVelocity(Supplier<AngularVelocity> targetVelocitySupplier, double timeSecondsForSetpoint) {
        io.setVelocity(targetVelocitySupplier.get(), timeSecondsForSetpoint);
    }

    public void applyJoystickInput() {
        double input = -MathUtil.copyDirectionPow(MathUtil.applyDeadband(RobotContainer.driverController.getRightY(), 0.1), 1.5);
        double voltage = input * RobotController.getBatteryVoltage();
        io.setVoltage(voltage);
    }

    public Command stopCommand() {
        return Commands.runOnce(() -> io.stop(), this);
    }

    public Command setVelocityCommand(AngularVelocity targetVelocity, double timeSecondsForSetpoint) {
        return Commands.run(() -> io.setVelocity(targetVelocity, timeSecondsForSetpoint));
    }

    public Command setVelocityCommand(Supplier<AngularVelocity> targetVelocitySupplier, double timeSecondsForSetpoint) {
        return Commands.run(() -> io.setVelocity(targetVelocitySupplier.get(), timeSecondsForSetpoint));
    }

    public Command joystickControlCommand() {
        return Commands.run(() -> applyJoystickInput(), this);
    }

}
