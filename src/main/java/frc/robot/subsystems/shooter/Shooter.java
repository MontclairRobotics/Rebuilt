package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.constants.HoodConstants;
import frc.robot.constants.TurretConstants;
import frc.robot.subsystems.shooter.flywheel.Flywheel;
import frc.robot.subsystems.shooter.hood.Hood;
import frc.robot.subsystems.shooter.spindexer.Spindexer;
import frc.robot.subsystems.shooter.turret.Turret;
import frc.robot.util.FieldConstants;

public class Shooter {
    private Hood hood;
    private Flywheel flywheel;
    private Turret turret;
    private Spindexer spindexer;
    public Shooter(Hood hood, Flywheel flywheel, Turret turret, Spindexer spindexer){
        this.hood = hood;
        this.flywheel = flywheel;
        this.turret = turret;
        this.spindexer = spindexer;
    }

    public Command aimToHubCommand(){
        return Commands.parallel(
            turret.setFieldRelativeAngleCommand(() -> turret.getAngleToHub()),
            hood.setAngleCommand(() -> hood.getAngleToHub())
        );
    }

    public Command aimToPointCommand(Translation2d point, double height){
        return Commands.parallel(
            turret.setFieldRelativeAngleCommand(() -> turret.getAngleToPoint(point)),
            hood.setAngleCommand(() -> hood.getAngleToPoint(point, height))
            );
    }
    public Command shootToHubCommand(AngularVelocity targetRPS){
        return Commands.parallel(
            turret.setFieldRelativeAngleCommand(() -> turret.getAngleToHub()),
            hood.setAngleCommand(() -> hood.getAngleToHub()),
            flywheel.holdVelocityCommand(targetRPS),
            spindexer.spinCommand()
        );
    }

    public Command shootToPointCommand(Translation2d point, double height, AngularVelocity targetRPS){
        return Commands.parallel(
            turret.setFieldRelativeAngleCommand(() -> turret.getAngleToPoint(point)),
            hood.setAngleCommand(() -> hood.getAngleToPoint(point, height)),
            flywheel.holdVelocityCommand(targetRPS),
            spindexer.spinCommand()
            );
    }


    public Command scoringCommand = Commands.sequence(
        turret.setFieldRelativeAngleCommand(() -> turret.getAngleToHub()),
        hood.setAngleCommand(() -> hood.getAngleToHub())
    );

    public Command ferryingLeftCommand = Commands.sequence(
        turret.setFieldRelativeAngleCommand(() -> turret.getAngleToPoint(FieldConstants.ferryWaypoints.FAR_FERRYING_LEFT_POINT)),
        hood.setAngleCommand(() -> hood.getAngleToPoint(FieldConstants.ferryWaypoints.FAR_FERRYING_LEFT_POINT, FieldConstants.ferryWaypoints.FAR_FERRYING_LEFT_HEIGHT))
    );

    public Command ferryRightCommand = Commands.sequence(
        turret.setFieldRelativeAngleCommand(() -> turret.getAngleToPoint(FieldConstants.ferryWaypoints.FAR_FERRYING_RIGHT_POINT)),
        hood.setAngleCommand(() -> hood.getAngleToPoint(FieldConstants.ferryWaypoints.FAR_FERRYING_RIGHT_POINT, FieldConstants.ferryWaypoints.FAR_FERRYING_RIGHT_HEIGHT))
    );

    public Command stowCommand = Commands.sequence(
        turret.setFieldRelativeAngleCommand(TurretConstants.MIN_ANGLE),
        hood.setAngleCommand(HoodConstants.MIN_ANGLE)
    );
}
