package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.constants.TurretConstants;
import frc.robot.subsystems.shooter.aiming.AimingConstants.SimShootingParameters;

public class LaunchFuelCommand extends Command {

    private final Supplier<SimShootingParameters> paramsSupplier;
    private final double fireRate; // shots per second
    private double lastShotTime = 0.0; // seconds

    public LaunchFuelCommand(Supplier<SimShootingParameters> paramsSupplier, double fireRate) {
        this.paramsSupplier = paramsSupplier;
        this.fireRate = fireRate;
        addRequirements(RobotContainer.shooter);
    }

    @Override
    public void execute() {
        if (RobotContainer.driverController.circle().getAsBoolean() && RobotContainer.shooter.atSetpoint()) {
            double currentTime = Timer.getFPGATimestamp();

            if (currentTime - lastShotTime >= 1.0 / fireRate) {
                lastShotTime = currentTime;

                SimShootingParameters params = paramsSupplier.get();

                Logger.recordOutput("launchFuelCommand()/Robot Relative Turret Angle", params.robotRelativeTurretAngle());
                Logger.recordOutput("launchFuelCommand()/Hood Angle", params.hoodAngle());
                Logger.recordOutput("launchFuelCommand()/Exit Velocity", params.exitVelocity());

                RobotContainer.fuelSim.launchFuel(
                    params.exitVelocity(),
                    Degrees.of(90).plus(params.hoodAngle()),
                    params.robotRelativeTurretAngle().plus(Radians.of(RobotContainer.drivetrain.getWrappedHeading().getRadians())),
                    TurretConstants.ORIGIN_TO_TURRET.getMeasureZ()
                );
            }
        }
    }

    @Override
    public boolean isFinished() {
        return false; // run until interrupted
    }
}
