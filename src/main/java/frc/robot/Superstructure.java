package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.shooter.hood.Hood;
import frc.robot.subsystems.shooter.turret.Turret;
import frc.robot.util.FieldConstants;
public class Superstructure {

    public static Turret turret;
    public static Hood hood;
    public CommandScheduler commandScheduler;

    public enum Modes {
        SCORING,
        FERRYING_LEFT,
        FERRYING_RIGHT,
        NO_ZONE
    }

    private Modes currentMode = Modes.NO_ZONE;
    private Modes lastMode = Modes.NO_ZONE;

    public void switchModeRedAlliance() {

        Translation2d pos = RobotContainer.turret.getFieldRelativePosition();

        if (pos.getX() <= FieldConstants.LinesVertical.starting) {
            currentMode = Modes.SCORING;
        }
        else if (pos.getX() <= FieldConstants.LinesVertical.neutralZoneNear) {
            currentMode = Modes.NO_ZONE;
        }
        // else if (pos.getY() <= FieldConstants.LinesHorizontal.center &&
        //          pos.getX() <= FieldConstants.LinesVertical.center) {
        //     currentMode = Modes.CLOSE_FERRYING_RIGHT;
        // }
        // else if (pos.getY() >= FieldConstants.LinesHorizontal.center &&
        //          pos.getX() <= FieldConstants.LinesVertical.center) {
        //     currentMode = Modes.CLOSE_FERRYING_LEFT;
        // }
        else if (pos.getY() <= FieldConstants.LinesHorizontal.center &&
                 pos.getX() <= FieldConstants.LinesVertical.neutralZoneFar) {
            currentMode = Modes.FERRYING_RIGHT;
        }
        else if (pos.getY() >= FieldConstants.LinesHorizontal.center &&
                 pos.getX() <= FieldConstants.LinesVertical.neutralZoneFar) {
            currentMode = Modes.FERRYING_LEFT;
        }
        else {
            currentMode = Modes.NO_ZONE;
        }
    }

    public void switchModeBlueAlliance() {

        Translation2d pos = RobotContainer.turret.getFieldRelativePosition();

        if (pos.getX() >= FieldConstants.LinesVertical.starting) {
            currentMode = Modes.SCORING;
        }
        else if (pos.getX() >= FieldConstants.LinesVertical.neutralZoneNear) {
            currentMode = Modes.NO_ZONE;
        }
        // else if (pos.getY() >= FieldConstants.LinesHorizontal.center &&
        //          pos.getX() >= FieldConstants.LinesVertical.center) {
        //     currentMode = Modes.CLOSE_FERRYING_RIGHT;
        // }
        // else if (pos.getY() <= FieldConstants.LinesHorizontal.center &&
        //          pos.getX() >= FieldConstants.LinesVertical.center) {
        //     currentMode = Modes.CLOSE_FERRYING_LEFT;
        // }
        else if (pos.getY() >= FieldConstants.LinesHorizontal.center &&
                 pos.getX() >= FieldConstants.LinesVertical.neutralZoneFar) {
            currentMode = Modes.FERRYING_RIGHT;
        }
        else if (pos.getY() <= FieldConstants.LinesHorizontal.center &&
                 pos.getX() >= FieldConstants.LinesVertical.neutralZoneFar) {
            currentMode = Modes.FERRYING_LEFT;
        }
        else {
            currentMode = Modes.NO_ZONE;
        }
    }

    public Modes getCurrentMode() {
        Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Red);

        if (alliance == Alliance.Red) {
            switchModeRedAlliance();
        } else {
            switchModeBlueAlliance();
        }
        return currentMode;
    }

    public void setModeConstants(){
        Turret turret = RobotContainer.turret;
        Hood hood = RobotContainer.hood;

        if (currentMode == lastMode) return;
        lastMode = currentMode;


        switch(currentMode){
            case NO_ZONE:
                break;
            case SCORING:
                CommandScheduler.getInstance().schedule(turret.setFieldRelativeAngleCommand(() -> turret.getAngleToHub()));
                CommandScheduler.getInstance().schedule(hood.setAngleCommand(() -> hood.getAngleToHub()));
                break;
            // case CLOSE_FERRYING_LEFT:
            //     CommandScheduler.getInstance().schedule(turret.setFieldRelativeAngleCommand(() -> turret.getAngleToPoint(FieldConstants.ferryWaypoints.CLOSE_FERRYING_LEFT_POINT)));
            //     CommandScheduler.getInstance().schedule(hood.setAngleCommand(() -> hood.getAngleToPoint(FieldConstants.ferryWaypoints.CLOSE_FERRYING_LEFT_POINT, FieldConstants.ferryWaypoints.CLOSE_FERRYING_LEFT_HEIGHT)));
            //     break;
            // case CLOSE_FERRYING_RIGHT:
            //     CommandScheduler.getInstance().schedule(turret.setFieldRelativeAngleCommand(() -> turret.getAngleToPoint(FieldConstants.ferryWaypoints.CLOSE_FERRYING_RIGHT_POINT)));
            //     CommandScheduler.getInstance().schedule(hood.setAngleCommand(() -> hood.getAngleToPoint(FieldConstants.ferryWaypoints.CLOSE_FERRYING_RIGHT_POINT, FieldConstants.ferryWaypoints.CLOSE_FERRYING_RIGHT_HEIGHT)));
            //     break;
            case FERRYING_LEFT:
                CommandScheduler.getInstance().schedule(turret.setFieldRelativeAngleCommand(() -> turret.getAngleToPoint(FieldConstants.ferryWaypoints.FAR_FERRYING_LEFT_POINT)));
                CommandScheduler.getInstance().schedule(hood.setAngleCommand(() -> hood.getAngleToPoint(FieldConstants.ferryWaypoints.FAR_FERRYING_LEFT_POINT, FieldConstants.ferryWaypoints.FAR_FERRYING_LEFT_HEIGHT)));
                break;
            case FERRYING_RIGHT:
                CommandScheduler.getInstance().schedule(turret.setFieldRelativeAngleCommand(() -> turret.getAngleToPoint(FieldConstants.ferryWaypoints.FAR_FERRYING_RIGHT_POINT)));
                CommandScheduler.getInstance().schedule(hood.setAngleCommand(() -> hood.getAngleToPoint(FieldConstants.ferryWaypoints.FAR_FERRYING_RIGHT_POINT, FieldConstants.ferryWaypoints.FAR_FERRYING_RIGHT_HEIGHT)));
                break;
        }
    }
}
