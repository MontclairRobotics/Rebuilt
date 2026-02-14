// package frc.robot.subsystems.shooter;

// import edu.wpi.first.math.geometry.Pose3d;
// import edu.wpi.first.math.geometry.Rotation3d;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.math.geometry.Translation3d;
// import static edu.wpi.first.units.Units.Rotations;
// import edu.wpi.first.units.measure.AngularVelocity;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.Commands;
// import frc.robot.constants.HoodConstants;
// import frc.robot.constants.TurretConstants;
// import frc.robot.subsystems.shooter.flywheel.Flywheel;
// import frc.robot.subsystems.shooter.hood.Hood;
// import frc.robot.subsystems.shooter.spindexer.Spindexer;
// import frc.robot.subsystems.shooter.turret.Turret;
// import frc.robot.util.FieldConstants;

// public class Shooter {
//     private Hood hood;
//     private Flywheel flywheel;
//     private Turret turret;
//     private Spindexer spindexer;
//     public Shooter(Hood hood, Flywheel flywheel, Turret turret, Spindexer spindexer){
//         this.hood = hood;
//         this.flywheel = flywheel;
//         this.turret = turret;
//         this.spindexer = spindexer;
//     }

// 	public Pose3d getFieldRelativePosition() {
// 		Translation2d turretTranslation2d = turret.getFieldRelativePosition();
// 		return new Pose3d(
// 			new Translation3d(
// 				turretTranslation2d.getX(),
// 				turretTranslation2d.getY(),
// 				TurretConstants.ORIGIN_TO_TURRET.getZ()
// 			),
// 			new Rotation3d(
// 				Rotations.zero(),
// 				Rotations.zero(),
// 				turret.getFieldRelativeAngle()
// 			)
// 		);
// 	}

//     public Command aimToHubCommand(){
//         return Commands.parallel(
//             turret.setFieldRelativeAngleCommand(() -> turret.getAngleToHub()),
//             hood.setAngleCommand(() -> hood.getAngleToHub())
//         );
//     }

//     public Command aimToPointCommand(Translation2d point, double height){
//         return Commands.parallel(
//             turret.setFieldRelativeAngleCommand(() -> turret.getAngleToPoint(point)),
//             hood.setAngleCommand(() -> hood.getAngleToPoint(point, height))
//             );
//     }
//     public Command shootToHubCommand(AngularVelocity targetRPS){
//         return Commands.parallel(
//             turret.setFieldRelativeAngleCommand(() -> turret.getAngleToHub()),
//             hood.setAngleCommand(() -> hood.getAngleToHub()),
//             flywheel.holdVelocityCommand(targetRPS),
//             spindexer.spinCommand()
//         );
//     }

//     public Command shootToPointCommand(Translation2d point, double height, AngularVelocity targetRPS){
//         return Commands.parallel(
//             turret.setFieldRelativeAngleCommand(() -> turret.getAngleToPoint(point)),
//             hood.setAngleCommand(() -> hood.getAngleToPoint(point, height)),
//             flywheel.holdVelocityCommand(targetRPS),
//             spindexer.spinCommand()
//             );
//     }


//     public Command scoringCommand(){
//         return Commands.parallel (
//         turret.setFieldRelativeAngleCommand(() -> turret.getAngleToHub()),
//         hood.setAngleCommand(() -> hood.getAngleToHub()));
//     }
//     ;

//     public Command ferryingLeftCommand() {
//         return Commands.parallel (
//         turret.setFieldRelativeAngleCommand(() -> turret.getAngleToPoint(FieldConstants.ferryWaypoints.LEFT_FERRYING_POINT)),
//         hood.setAngleCommand(() -> hood.getAngleToPoint(FieldConstants.ferryWaypoints.LEFT_FERRYING_POINT, FieldConstants.ferryWaypoints.LEFT_FERRYING_HEIGHT)));
//     };

//     public Command ferryRightCommand(){
//         return Commands.parallel (
//         turret.setFieldRelativeAngleCommand(() -> turret.getAngleToPoint(FieldConstants.ferryWaypoints.RIGHT_FERRYING_POINT)),
//         hood.setAngleCommand(() -> hood.getAngleToPoint(FieldConstants.ferryWaypoints.RIGHT_FERRYING_POINT, FieldConstants.ferryWaypoints.RIGHT_FERRYING_HEIGHT)));
//     };

//     public Command stowCommand(){
//         return Commands.parallel (
// 			hood.setAngleCommand(() -> HoodConstants.MIN_ANGLE),
// 			turret.stopCommand()
// 		);
//     }
// }
