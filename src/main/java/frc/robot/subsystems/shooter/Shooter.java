package frc.robot.subsystems.shooter;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;

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

    private Hood hood;
    private Flywheel flywheel;
    private Turret turret;
    private Spindexer spindexer;

    private boolean whileMoving;

    private final int FIRE_RATE = 6;
    private double lastSimShotTime = 0.0;

    // prevents feeding on a momentary false positive
    private Debouncer setpointDebouncer = new Debouncer(0.04, DebounceType.kRising);

    // lets the spindexer keep running through the RPM dip after a shot
    private Debouncer feedThroughDebouncer = new Debouncer(1, DebounceType.kFalling);

    public Shooter(Hood hood, Flywheel flywheel, Turret turret, Spindexer spindexer, boolean whileMoving) {
        this.hood = hood;
        this.flywheel = flywheel;
        this.turret = turret;
        this.spindexer = spindexer;
        this.whileMoving = whileMoving;
    }

    @Override
    public void periodic() {
        Logger.recordOutput("Shooter/At Setpoint", atSetpoint());
    }

    public boolean atSetpoint() {
        return feedThroughDebouncer.calculate(
            setpointDebouncer.calculate(
                turret.atSetpoint() && hood.atSetpoint() && (flywheel.atSetpoint() || RobotBase.isSimulation())
            )
        );
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

    public Command shootStaticallyCommand() {
        return Commands.runEnd(
            () -> {

                ShootingParameters params = new ShootingParameters(
                    Rotations.zero(),
                    Degrees.of(10),
                    RotationsPerSecond.of(24),
                    Timer.getFPGATimestamp()
                );

                turret.setRobotRelativeAngle(params.robotRelativeTurretAngle(), RotationsPerSecond.zero());
                hood.setAngle(params.hoodAngle());
                flywheel.setVelocity(params.flywheelVelocity());
                spindexer.spinUp();

            },
            () -> {
                flywheel.stop();
                turret.stop();
                hood.setAngle(HoodConstants.MIN_ANGLE);
                spindexer.stop();
            },
            this, flywheel, turret, hood, spindexer
        );
    }

    public Command applyShooterGoalCommand(Supplier<ShooterGoal> goalSupplier) {
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

    public Command applySimShooterGoalCommand(Supplier<ShooterGoal> goalSupplier) {
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
        if (this.atSetpoint() && !RobotContainer.turret.isSpinningAround()) {
            double currentTime = Timer.getFPGATimestamp();
            double interval = 1.0 / fireRate;

            if(currentTime - lastSimShotTime >= interval) {
                lastSimShotTime = currentTime;

                LinearVelocity exitVelocity = velocitySupplier.get().times(1 + ((Math.random() * 0.05)-0.025));
                Angle robotRelativeTurretAngle = turret.getRobotRelativeAngle();
                Angle hoodAngle = hood.getAngle();

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
            return applyShooterGoalCommand(() -> RobotContainer.shooterCoordinator.getCurrentGoal());
        } else {
            return applySimShooterGoalCommand(() -> RobotContainer.shooterCoordinator.getCurrentGoal());
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
