package frc.robot.subsystems.intake;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.constants.PivotConstants;
import frc.robot.subsystems.intake.pivot.Pivot;
import frc.robot.subsystems.intake.rollers.Rollers;

public class Intake {
    private Pivot pivot;
    private Rollers rollers;
    public Intake(Pivot pivot, Rollers rollers){
        this.pivot = pivot;
        this.rollers = rollers;
        
    }

    public Command stopCommand(){
        return Commands.parallel(
            rollers.stopCommand(),
            pivot.stopCommand()
        );
    }

    public Command intakeCommand(){
        return Commands.parallel(
            pivot.goToAngleCommand(PivotConstants.MIN_ANGLE),
            rollers.intakeCommand()
        );
    }

    public Command wiggleCommand(){
        return Commands.sequence(
            rollers.intakeCommand().withTimeout(0.13),
            rollers.outtakeCommand().withTimeout(0.13),
            rollers.intakeCommand().withTimeout(0.13)
        );
    }
}
