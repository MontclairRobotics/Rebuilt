package frc.robot.subsystems.intake;

import static frc.robot.constants.PivotConstants.*;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
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

    public Command jostleCommand() {
        return Commands.repeatingSequence(
            pivot.goToAngleCommand(MAX_ANGLE.div(2)),
            pivot.goToAngleCommand(MIN_ANGLE)
        ).withInterruptBehavior(InterruptionBehavior.kCancelSelf)
        .finallyDo(() -> pivot.stopCommand());
    }

    public Command stopCommand() {
        return Commands.parallel(
            rollers.spinDownCommand(),
            pivot.stopCommand()
        );
    }

    public Command intakeCommand() {
        return Commands.parallel(
            pivot.goToAngleCommand(PivotConstants.MIN_ANGLE),
            rollers.spinUpCommand()
        );
    }

}
