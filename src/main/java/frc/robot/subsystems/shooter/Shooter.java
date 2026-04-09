package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.constants.HoodConstants;
import frc.robot.constants.TurretConstants;
import frc.robot.subsystems.shooter.ShooterCoordinator.ShooterGoal;
import frc.robot.subsystems.shooter.aiming.Aiming;
import frc.robot.subsystems.shooter.aiming.Aiming.TargetLocation;
import frc.robot.subsystems.shooter.aiming.AimingConstants.ShootingParameters;
import frc.robot.subsystems.shooter.aiming.AimingConstants.SimShootingParameters;
import frc.robot.subsystems.shooter.flywheel.Flywheel;
import frc.robot.subsystems.shooter.turret.Turret;
import frc.robot.subsystems.shooter.hood.Hood;
import frc.robot.subsystems.shooter.spindexer.Spindexer;

public class Shooter extends SubsystemBase {

    public static TargetLocation targetLocation;

    public Hood hood;
    public Flywheel flywheel;
    public Turret turret;
    public Spindexer spindexer;

    public boolean withConstantVelocity;
    public boolean whileMoving;

    public final int HOPPER_CAPACITY = 40;
    private final int FIRE_RATE = 6;
    public int hopperCount;

    private double lastSimShotTime = 0.0;

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
        Logger.recordOutput("Shooter/At Setpoint", atSetpoint());
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
        double intakeProbability = Math.max(0, 1 - RobotContainer.drivetrain.getFieldRelativeLinearVelocity().in(MetersPerSecond) / 3);
        return hopperCount < HOPPER_CAPACITY
            && Math.random() < intakeProbability;
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
        return turret.atSetpoint() && hood.atSetpoint() && (flywheel.atSetpoint() || RobotBase.isSimulation());
    }

    public TargetLocation getTargetFromGoal(ShooterGoal goal) {
        TargetLocation target = null;

        switch (goal.mode()) {
            case SCORING:
                target = TargetLocation.HUB;
                break;
            case FERRYING_LEFT:
                target = TargetLocation.FERRY_LEFT;
                break;
            case FERRYING_RIGHT:
                target = TargetLocation.FERRY_RIGHT;
                break;
            case IDLE:
                break;
        };

        return target;
    }

    public AngularVelocity getTargetTurretVelocity(ShooterGoal goal) {
        TargetLocation target = getTargetFromGoal(goal);

        if(target != null) {
            return turret.calculateTargetVelocity(target);
        } else {
            return RotationsPerSecond.zero();
        }
    }

    public ShootingParameters getShootingParameters(ShooterGoal goal) {
        TargetLocation target = getTargetFromGoal(goal);

        if(target != null)  {
            return Aiming.calculateShot(
                target,
                withConstantVelocity,
                whileMoving
            );
        } else {
            return new ShootingParameters(
                Rotations.zero(),
                Rotations.zero(),
                RotationsPerSecond.zero(),
                Timer.getFPGATimestamp()
            );
        }

    }

    public SimShootingParameters getSimShootingParameters(ShooterGoal goal) {
        TargetLocation target = getTargetFromGoal(goal);

        if(target != null)  {
            return Aiming.calculateSimShot(
                target,
                withConstantVelocity,
                whileMoving
            );
        } else {
            return new SimShootingParameters(
                Rotations.zero(),
                Rotations.zero(),
                MetersPerSecond.zero()
            );
        }
    }

    public Command getShooterGoalCommand(Supplier<ShooterGoal> goalSupplier) {
        return Commands.runEnd(() -> {

            ShooterGoal goal = goalSupplier.get();
            ShootingParameters params = getShootingParameters(goal);
            AngularVelocity targetTurretVelocity = getTargetTurretVelocity(goal);

            if(goal.intent().isToSpinFlywheel()) {
                flywheel.setVelocity(params.flywheelVelocity());
            } else {
                flywheel.stop();
            }

            if(goal.intent().isToUseTurretAngle()) {
                turret.setRobotRelativeAngle(params.robotRelativeTurretAngle(), targetTurretVelocity);
            } else {
                turret.stop();
            }

            if(goal.intent().isToUseHoodAngle()) {
                hood.setAngle(params.hoodAngle());
            } else {
                hood.setAngle(HoodConstants.MIN_ANGLE);
            }

            if(goal.intent().isToFeedBalls() && this.atSetpoint()) {
                spindexer.spinUp();
            } else {
                spindexer.stop();
            }

        },
        () -> {
            flywheel.stop();
            turret.stop();
            hood.setAngle(HoodConstants.MIN_ANGLE);
            spindexer.stop();
        },
        this, flywheel, turret, hood, spindexer);
    }

    public Command getSimShooterGoalCommand(Supplier<ShooterGoal> goalSupplier) {
        return Commands.runEnd(() -> {

            ShooterGoal goal = goalSupplier.get();
            SimShootingParameters params = getSimShootingParameters(goal);
            AngularVelocity targetTurretVelocity = getTargetTurretVelocity(goal);

            if(goal.intent().isToUseTurretAngle()) {
                turret.setRobotRelativeAngle(params.robotRelativeTurretAngle(), targetTurretVelocity);
            } else {
                turret.stop();
            }

            if(goal.intent().isToUseHoodAngle()) {
                hood.setAngle(params.hoodAngle());
            } else {
                hood.setAngle(HoodConstants.MIN_ANGLE);
            }

            if(goal.intent().isToFeedBalls() && this.atSetpoint()) {
                launchFuel(() -> params.exitVelocity(), FIRE_RATE);
            } else {
                // nothing needed
            }

        },
        () -> {
            flywheel.stop();
            turret.stop();
            hood.setAngle(HoodConstants.MIN_ANGLE);
            spindexer.stop();
        },
        this, flywheel, turret, hood, spindexer);
    }

    public void launchFuel(Supplier<LinearVelocity> velocitySupplier, double fireRate) {
        if (RobotContainer.shooter.atSetpoint() && !Turret.isSpinningAround) {
            double currentTime = Timer.getFPGATimestamp();
            double interval = 1.0 / fireRate;

            if(currentTime - lastSimShotTime >= interval) {
                lastSimShotTime = currentTime;
                removeBall();

                LinearVelocity exitVelocity = velocitySupplier.get().times(1 + ((Math.random() * 0.05)-0.025));
                Angle robotRelativeTurretAngle = turret.getRobotRelativeAngle();
                Angle hoodAngle = hood.getAngle();

                Logger.recordOutput("launchFuelCommand()/Robot Relative Turret Angle", robotRelativeTurretAngle.in(Rotations));
                Logger.recordOutput("launchFuelCommand()/Hood Angle", hoodAngle.in(Rotations));
                Logger.recordOutput("launchFuelCommand()/Exit Velocity", exitVelocity.in(MetersPerSecond));

                RobotContainer.fuelSim.launchFuel(
                    exitVelocity,
                    Degrees.of(90).plus(hoodAngle),
                    robotRelativeTurretAngle.plus(Radians.of(RobotContainer.drivetrain.getWrappedHeading().getRadians())).minus(Rotations.of(0)),
                    TurretConstants.ORIGIN_TO_TURRET.getMeasureZ()
                );
            }
        }
    }

    public Command getDefaultCommand() {
        if(RobotBase.isReal()) {
            return getShooterGoalCommand(() -> ShooterCoordinator.currentGoal);
        } else {
            return getSimShooterGoalCommand(() -> ShooterCoordinator.currentGoal);
        }
    }

    public Command stowCommand(){
        return Commands.parallel (
			hood.setAngleCommand(HoodConstants.MIN_ANGLE),
			turret.stopCommand(),
            flywheel.stopCommand(),
            spindexer.stopCommand()
		);
    }

    public Command startShootingInAuto() {
        return Commands.runOnce(() -> {
            RobotContainer.shouldShootAuto = true;
        });
    }

    public Command stopShootingInAuto() {
        return Commands.runOnce(() -> {
            RobotContainer.shouldShootAuto = false;
        });
    }

}
