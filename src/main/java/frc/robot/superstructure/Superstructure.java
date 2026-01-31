package frc.robot.superstructure;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.RobotContainer;
import frc.robot.subsystems.turret.Turret;
import frc.robot.util.FieldConstants;
public class Superstructure {

    public static Turret turret;

    public enum Modes {
        SCORING,
        CLOSE_FERRYING_LEFT,
        CLOSE_FERRYING_RIGHT,
        FAR_FERRYING_LEFT,
        FAR_FERRYING_RIGHT,
        NO_ZONE
    }

    private Modes currentMode = Modes.NO_ZONE;

    public void switchMode() {

        Translation2d pos = RobotContainer.turret.getFieldRelativePosition();

        if (pos.getX() <= FieldConstants.LinesVertical.starting) { 
            currentMode = Modes.SCORING;   
        }
        else if (pos.getX() <= FieldConstants.LinesVertical.neutralZoneNear) {
            currentMode = Modes.NO_ZONE;
        }
        else if (pos.getY() <= FieldConstants.LinesHorizontal.center &&
                 pos.getX() <= FieldConstants.LinesVertical.center) {
            currentMode = Modes.CLOSE_FERRYING_RIGHT;
        }
        else if (pos.getY() >= FieldConstants.LinesHorizontal.center &&
                 pos.getX() <= FieldConstants.LinesVertical.center) {
            currentMode = Modes.CLOSE_FERRYING_LEFT;
        }
        else if (pos.getY() <= FieldConstants.LinesHorizontal.center &&
                 pos.getX() <= FieldConstants.LinesVertical.neutralZoneFar) {
            currentMode = Modes.FAR_FERRYING_RIGHT;
        }
        else if (pos.getY() >= FieldConstants.LinesHorizontal.center &&
                 pos.getX() <= FieldConstants.LinesVertical.neutralZoneFar) {
            currentMode = Modes.FAR_FERRYING_LEFT;
        }
        else {
            currentMode = Modes.NO_ZONE;
        }
    }

    public Modes getCurrentMode() {
        return currentMode;
    }
    public void setModeConstants(){
        switch(currentMode){
            case NO_ZONE:
                break;
            case SCORING:
                turret.setFieldRelativeAngleCommand(() -> turret.getAngleToHub());
                break;
            case CLOSE_FERRYING_LEFT:
                turret.setFieldRelativeAngleCommand(() -> turret.getAngleToPoint(FieldConstants.ferryWaypoints.CLOSE_FERRYING_LEFT_POINT));
                break;
            case CLOSE_FERRYING_RIGHT:
                turret.setFieldRelativeAngleCommand(() -> turret.getAngleToPoint(FieldConstants.ferryWaypoints.CLOSE_FERRYING_RIGHT_POINT));
                break;
            case FAR_FERRYING_LEFT:
                turret.setFieldRelativeAngleCommand(() -> turret.getAngleToPoint(FieldConstants.ferryWaypoints.FAR_FERRYING_LEFT_POINT));
                break;
            case FAR_FERRYING_RIGHT:
                turret.setFieldRelativeAngleCommand(() -> turret.getAngleToPoint(FieldConstants.ferryWaypoints.FAR_FERRYING_RIGHT_POINT));
                break;
        }
    }
}
