package frc.robot.superstructure;

import frc.robot.subsystems.turret.Turret;
import frc.robot.util.FieldConstants; 

public enum Modes{
    SCORING,
    CLOSE_FERRYING_LEFT,
    CLOSE_FERRYING_RIGHT,
    FAR_FERRYING_LEFT,
    FAR_FERRYING_RIGHT,
    NO_ZONE;
}

public void Switch(){
if (Turret.getFieldRelativePosition().getX() <= FieldConstants.LinesHorizontal.starting){ 
    Modes = Modes.SCORING;   
}
else if(Turret.getFieldRelativePosition().getX() <= FieldConstants.LinesHorizontal.neutralZoneNear){
    Modes = Modes.NO_ZONE;
}
else if (Turret.getFieldRelativePosition().getY() <= FieldConstants.LinesVertical.center && Turret.getFieldRelativePosition().getX() <= FieldConstants.LinesHorizontal.center){
    Modes = Modes.CLOSE_FERRYING_RIGHT;
}
else if (Turret.getFieldRelativePosition().getY() >= FieldConstants.LinesVertical.center && Turret.getFieldRelativePosition().getX() <= FieldConstants.LinesHorizontal.center){
    Modes = Modes.CLOSE_FERRYING_LEFT;
}
else if (Turret.getFieldRelativePosition().getY() <= FieldConstants.LinesVertical.center && Turret.getFieldRelativePosition().getX() <= FieldConstants.LinesHorizontal.neutralZoneFar){
    Modes = Modes.FAR_FERRYING_RIGHT;
}
else if (Turret.getFieldRelativePosition().getY() >= FieldConstants.LinesVertical.center && Turret.getFieldRelativePosition().getX() <= FieldConstants.LinesHorizontal.neutralZoneFar){
    Modes = Modes.FAR_FERRYING_LEFT;
}
else{
    Modes = Modes.NO_ZONE;
}
}