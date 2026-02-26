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
import frc.robot.subsystems.shooter.turret.Turret;
import frc.robot.util.HubTracker;
import frc.robot.util.sim.FuelSim.Hub;
import frc.robot.subsystems.shooter.hood.Hood;
import frc.robot.subsystems.shooter.spindexer.Spindexer;

public class Shooter extends SubsystemBase {

    private Hood hood;
    private Flywheel flywheel;
    private Turret turret;
    private Spindexer spindexer;

    public boolean withConstantVelocity;
    public boolean whileMoving;

    public final int HOPPER_CAPACITY = 40;
    private final int FIRE_RATE = 6;
    public int hopperCount;

    private  double lastSimShotTime = 0.0;

    public Shooter(Hood hood, Flywheel flywheel, Turret turret, Spindexer spindexer, boolean withConstantVelocity, boolean whileMoving) {
        this.hood = hood;
        this.flywheel = flywheel;
        this.turret = turret;
        this.spindexer = spindexer;
        this.withConstantVelocity = withConstantVelocity;
        this.whileMoving = whileMoving;
        this.hopperCount = 0;
    }


    @Override
    public void periodic() {
        Logger.recordOutput("Fuel/Hopper Count", hopperCount);
        Logger.recordOutput("Fuel/Blue Score", Hub.BLUE_HUB.getScore());
        Logger.recordOutput("Fuel/Red Score", Hub.RED_HUB.getScore());
        Logger.recordOutput("Hub/Match Time", HubTracker.getMatchTime());
    }

    public int getHopperCount() {
        return hopperCount;
    }

    public void addBall() {
        if (hopperCount < HOPPER_CAPACITY) {
            hopperCount++;
        }
    }

    public void removeBall() {
        if (hopperCount > 0) {
            hopperCount--;
        }
    }

    public boolean shouldIntake() {
        double intakeProbability = Math.max(0, 1 - RobotContainer.drivetrain.getFieldRelativeLinearSpeed().in(MetersPerSecond) / 4);
        return hopperCount < HOPPER_CAPACITY
            && Math.random() < intakeProbability
            && RobotContainer.pivot.io.getAngle().in(Degrees) < 3
            && RobotContainer.operatorController.L1().getAsBoolean();
    }

    public boolean hasBalls() {
        return hopperCount > 0;
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
        return turret.atSetpoint() && hood.atSetpoint();
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
                launchFuel(() -> params.exitVelocity(), FIRE_RATE);
                Logger.recordOutput("setSimParameters()/Robot Relative Turret Angle", params.robotRelativeTurretAngle().in(Rotations));
                Logger.recordOutput("setSimParameters()/Hood Angle", params.hoodAngle().in(Rotations));
                Logger.recordOutput("setSimParameters()/Exit Velocity", params.exitVelocity().in(MetersPerSecond));
                turret.setRobotRelativeAngle(() -> params.robotRelativeTurretAngle());
                hood.setAngle(() -> params.hoodAngle());
            })
        );
    }

    public void launchFuel(Supplier<LinearVelocity> velocitySupplier, double fireRate) {
        if (RobotContainer.shotTrigger.getAsBoolean() && RobotContainer.shooter.atSetpoint() && hasBalls()) {
            double currentTime = Timer.getFPGATimestamp();
            double interval = 1.0 / fireRate;

            if(currentTime - lastSimShotTime >= interval) {
                lastSimShotTime = currentTime;
                removeBall();

                LinearVelocity exitVelocity = velocitySupplier.get().times(1 + ((Math.random() * 0.05)-0.025));
                Angle robotRelativeTurretAngle = RobotContainer.turret.getRobotRelativeAngle();
                Angle hoodAngle = RobotContainer.hood.getAngle();

                Logger.recordOutput("launchFuelCommand()/Robot Relative Turret Angle", robotRelativeTurretAngle.in(Rotations));
                Logger.recordOutput("launchFuelCommand()/Hood Angle", hoodAngle.in(Rotations));
                Logger.recordOutput("launchFuelCommand()/Exit Velocity", exitVelocity.in(MetersPerSecond));

                RobotContainer.fuelSim.launchFuel(
                    exitVelocity,
                    Degrees.of(90).plus(hoodAngle),
                    robotRelativeTurretAngle.plus(Radians.of(RobotContainer.drivetrain.getWrappedHeading().getRadians())).minus(Rotations.of(0.125)),
                    TurretConstants.ORIGIN_TO_TURRET.getMeasureZ()
                );
            }
        }
    }

    public Command indexAndShootCommand(Supplier<AngularVelocity> flywheelVelocitySupplier) {
        return Commands.run(() -> {
            if (RobotContainer.driverController.R2().getAsBoolean() && this.atSetpoint()) {
                spindexer.spinUp();
                flywheel.setVelocity(flywheelVelocitySupplier);
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
