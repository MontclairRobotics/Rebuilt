package frc.robot.superstructure;

import edu.wpi.first.math.geometry.Translation2d;

public class Superstructure {

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

        Translation2d pos = Turret.getFieldRelativePosition();

        if (pos.getX() <= FieldConstants.LinesHorizontal.starting) { 
            currentMode = Modes.SCORING;   
        }
        else if (pos.getX() <= FieldConstants.LinesHorizontal.neutralZoneNear) {
            currentMode = Modes.NO_ZONE;
        }
        else if (pos.getY() <= FieldConstants.LinesVertical.center &&
                 pos.getX() <= FieldConstants.LinesHorizontal.center) {
            currentMode = Modes.CLOSE_FERRYING_RIGHT;
        }
        else if (pos.getY() >= FieldConstants.LinesVertical.center &&
                 pos.getX() <= FieldConstants.LinesHorizontal.center) {
            currentMode = Modes.CLOSE_FERRYING_LEFT;
        }
        else if (pos.getY() <= FieldConstants.LinesVertical.center &&
                 pos.getX() <= FieldConstants.LinesHorizontal.neutralZoneFar) {
            currentMode = Modes.FAR_FERRYING_RIGHT;
        }
        else if (pos.getY() >= FieldConstants.LinesVertical.center &&
                 pos.getX() <= FieldConstants.LinesHorizontal.neutralZoneFar) {
            currentMode = Modes.FAR_FERRYING_LEFT;
        }
        else {
            currentMode = Modes.NO_ZONE;
        }
    }

    public Modes getCurrentMode() {
        return currentMode;
    }
}
