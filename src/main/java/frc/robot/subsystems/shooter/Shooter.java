package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.constants.HoodConstants;
import frc.robot.constants.TurretConstants;
import frc.robot.subsystems.shooter.aiming.AimingConstants.ShootingParameters;
import frc.robot.subsystems.shooter.aiming.AimingConstants.SimShootingParameters;
import frc.robot.subsystems.shooter.flywheel.Flywheel;
import frc.robot.subsystems.shooter.hood.Hood;
import frc.robot.subsystems.shooter.spindexer.Spindexer;
import frc.robot.subsystems.shooter.turret.Turret;

public class Shooter extends SubsystemBase {

    private Hood hood;
    private Flywheel flywheel;
    private Turret turret;
    private Spindexer spindexer;

    public boolean withConstantVelocity;
    public boolean whileMoving;

    private  double lastSimShotTime = 0.0;

    public Shooter(Hood hood, Flywheel flywheel, Turret turret, Spindexer spindexer, boolean withConstantVelocity, boolean whileMoving) {
        this.hood = hood;
        this.flywheel = flywheel;
        this.turret = turret;
        this.spindexer = spindexer;
        this.withConstantVelocity = withConstantVelocity;
        this.whileMoving = whileMoving;
    }

	public Pose3d getFieldRelativePosition() {
		Translation2d turretTranslation2d = turret.getFieldRelativePosition();
		return new Pose3d(
			new Translation3d(
				turretTranslation2d.getX(),
				turretTranslation2d.getY(),
				TurretConstants.ORIGIN_TO_TURRET.getZ()
			),
			new Rotation3d(
				Rotations.zero(),
				Rotations.zero(),
				turret.getFieldRelativeAngle()
			)
		);
	}

    public boolean atSetpoint() {
        return turret.atSetpointForShooting() && hood.atSetpoint();
    }

    public Command setParameters(Supplier<ShootingParameters> paramsSupplier) {
        return Commands.parallel(
            turret.setRobotRelativeAngleCommand(() -> paramsSupplier.get().robotRelativeTurretAngle()),
            hood.setAngleCommand(() -> paramsSupplier.get().hoodAngle()),
            indexAndShootCommand(() -> paramsSupplier.get().flywheelVelocity())
        );
    }

    public Command setSimParameters(Supplier<SimShootingParameters> paramsSupplier) {
        return Commands.parallel(
            Commands.run(() -> {
                SimShootingParameters params = paramsSupplier.get();
                Logger.recordOutput("launchFuel()/At Setpoint", RobotContainer.shooter.atSetpoint());
                launchFuel(() -> params.exitVelocity(), 6);
                Logger.recordOutput("setSimParameters()/Robot Relative Turret Angle", params.robotRelativeTurretAngle().in(Rotations));
                Logger.recordOutput("setSimParameters()/Hood Angle", params.hoodAngle().in(Rotations));
                Logger.recordOutput("setSimParameters()/Exit Velocity", params.exitVelocity().in(MetersPerSecond));
                turret.setRobotRelativeAngle(() -> params.robotRelativeTurretAngle());
                hood.setAngle(() -> params.hoodAngle());
            })
        );
    }

    public void launchFuel(Supplier<LinearVelocity> velocitySupplier, double fireRate) {
        if (RobotContainer.driverController.R2().getAsBoolean() && RobotContainer.shooter.atSetpoint()) {
            double currentTime = Timer.getFPGATimestamp();
            double interval = 1.0 / fireRate;

            if(currentTime - lastSimShotTime >= interval) {
                lastSimShotTime = currentTime;

                LinearVelocity exitVelocity = velocitySupplier.get();
                Angle robotRelativeTurretAngle = RobotContainer.turret.getRobotRelativeAngle();
                Angle hoodAngle = RobotContainer.hood.getAngle();

                Logger.recordOutput("launchFuelCommand()/Robot Relative Turret Angle", robotRelativeTurretAngle.in(Rotations));
                Logger.recordOutput("launchFuelCommand()/Hood Angle", hoodAngle.in(Rotations));
                Logger.recordOutput("launchFuelCommand()/Exit Velocity", exitVelocity.in(MetersPerSecond));

                RobotContainer.fuelSim.launchFuel(
                    exitVelocity,
                    Degrees.of(90).plus(hoodAngle),
                    robotRelativeTurretAngle.plus(Radians.of(RobotContainer.drivetrain.getWrappedHeading().getRadians())),
                    TurretConstants.ORIGIN_TO_TURRET.getMeasureZ()
                );
            }
        }
    }

    public Command indexAndShootCommand(Supplier<AngularVelocity> flywheelVelocitySupplier) {
        return Commands.run(() -> {
            if (RobotContainer.driverController.R2().getAsBoolean() && this.atSetpoint()) {
                spindexer.spin();
                flywheel.setVelocityRPS(flywheelVelocitySupplier);
            }
        });
    }

    public Command stowCommand(){
        return Commands.parallel (
			hood.setAngleCommand(() -> HoodConstants.MIN_ANGLE),
			turret.stopCommand()
		);
    }
}
