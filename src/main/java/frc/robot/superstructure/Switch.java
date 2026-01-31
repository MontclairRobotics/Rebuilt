package frc.robot.superstructure;
import frc.robot.superstructure.Modes;

public class Switch{
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