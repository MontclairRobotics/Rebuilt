package frc.robot.subsystems.shooter;

import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Superstructure;
import frc.robot.subsystems.shooter.aiming.Aiming;

public class ShooterCoordinator extends SubsystemBase{

	private ShooterGoal currentGoal;

	private BooleanSupplier wantsToFire;
	private BooleanSupplier wantsToTrackHub;
	private BooleanSupplier wantsToTrackFerry;

	private BooleanSupplier shouldShootAuto;

    public enum ShooterIntent {

		INACTIVE(
			false,
			false,
			false,
			false
		),
		ARMED(
			true,
			true,
			false,
			false
		),
		FIRING(
			true,
			true,
			true,
			true
		);

		private boolean spinFlywheel;
		private boolean useTurretAngle;
		private boolean useHoodAngle;
		private boolean feedBalls;

		ShooterIntent(boolean spinFlywheel, boolean useTurretAngle, boolean useHoodAngle, boolean feedBalls) {
			this.spinFlywheel = spinFlywheel;
			this.useTurretAngle = useTurretAngle;
			this.useHoodAngle = useHoodAngle;
			this.feedBalls = feedBalls;
		}

		public boolean isToSpinFlywheel() {return spinFlywheel;}
		public boolean isToUseTurretAngle() {return useTurretAngle;}
		public boolean isToUseHoodAngle() {return useHoodAngle;}
		public boolean isToFeedBalls() {return feedBalls;}

	}

	public enum ShooterMode {

		IDLE(Translation2d.kZero),
		SCORING(Aiming.TargetLocation.HUB.getLocation()),
		FERRYING_RIGHT(Aiming.TargetLocation.FERRY_RIGHT.getLocation()),
		FERRYING_LEFT(Aiming.TargetLocation.FERRY_LEFT.getLocation());

		private Translation2d pointOfInterest;

		ShooterMode(Translation2d pointOfInterest) {
			this.pointOfInterest = pointOfInterest;
		}

		public Translation2d pointOfInterest() {
            return pointOfInterest;
        }
	}

    public record ShooterGoal(ShooterMode mode, ShooterIntent intent) {
        public ShooterGoal(ShooterMode mode, ShooterIntent intent) {
            this.mode = mode;
            this.intent = intent;
        }
    }

    public ShooterCoordinator(
		BooleanSupplier wantsToFire,
		BooleanSupplier wantsToTrackHub,
		BooleanSupplier wantsToTrackFerry,
		BooleanSupplier shouldShootAuto
	) {
		currentGoal = new ShooterGoal(ShooterMode.IDLE, ShooterIntent.INACTIVE);
		this.wantsToFire = wantsToFire;
		this.wantsToTrackHub = wantsToTrackHub;
		this.wantsToTrackFerry = wantsToTrackFerry;
		this.shouldShootAuto = shouldShootAuto;
    }

	public ShooterGoal getCurrentGoal() {return currentGoal;}
	public ShooterMode getCurrentMode() {return currentGoal.mode();}
	public ShooterIntent getCurrentIntent() {return currentGoal.intent();}

    public ShooterGoal calculateGoal(
		boolean inScoringZone,
		boolean inLeftFerryZone,
		boolean inRightFerryZone,
		boolean inTrenchDangerZone,
		boolean operatorWantsToTrackHub,
		boolean operatorWantsToTrackFerryPoint,
		boolean operatorWantsToFire,
		boolean shouldShootAuto
	) {

		if(inTrenchDangerZone) {
			// Only preserve mode if we were actually tracking something
    		ShooterMode safeMode = currentGoal.mode() == ShooterMode.IDLE
        		? ShooterMode.IDLE
        		: currentGoal.mode();
    		ShooterIntent safeIntent = currentGoal.mode() == ShooterMode.IDLE
        		? ShooterIntent.INACTIVE
        		: ShooterIntent.ARMED;
			return new ShooterGoal(safeMode, safeIntent);
		}

		// if we're in auto, we force scoring but only FIRE while in the alliance zone
		if(shouldShootAuto) {
			if(inScoringZone) {
				return new ShooterGoal(ShooterMode.SCORING, ShooterIntent.FIRING);
			} else {
				return new ShooterGoal(ShooterMode.SCORING, ShooterIntent.ARMED);
			}
		}

		// when the operator wants to track the hub
		if(operatorWantsToTrackHub) {
			if(operatorWantsToFire && inScoringZone) {
				return new ShooterGoal(ShooterMode.SCORING, ShooterIntent.FIRING);
			} else {
				return new ShooterGoal(ShooterMode.SCORING, ShooterIntent.ARMED);
			}
		}

		// when the operator wants to ferry
		if(operatorWantsToTrackFerryPoint){
			if(inLeftFerryZone) {
				if(operatorWantsToFire) {
					return new ShooterGoal(ShooterMode.FERRYING_LEFT, ShooterIntent.FIRING);
				} else {
					return new ShooterGoal(ShooterMode.FERRYING_LEFT, ShooterIntent.ARMED);
				}
			} else if(inRightFerryZone) {
				if(operatorWantsToFire) {
					return new ShooterGoal(ShooterMode.FERRYING_RIGHT, ShooterIntent.FIRING);
				} else {
					return new ShooterGoal(ShooterMode.FERRYING_RIGHT, ShooterIntent.ARMED);
				}
			} else {
				return new ShooterGoal(ShooterMode.IDLE, ShooterIntent.INACTIVE);
			}
		}

		return new ShooterGoal(ShooterMode.IDLE, ShooterIntent.INACTIVE);
	}

	@Override
	public void periodic(){

		currentGoal = calculateGoal(
			Superstructure.inScoringZone(),
			Superstructure.inLeftFerryZone(),
			Superstructure.inRightFerryZone(),
			Superstructure.inTrenchDangerZone(),
			wantsToTrackHub.getAsBoolean(),
			wantsToTrackFerry.getAsBoolean(),
			wantsToFire.getAsBoolean(),
			shouldShootAuto.getAsBoolean()
		);

		Logger.recordOutput("Shooter/GoalMode", currentGoal.mode().toString());
		Logger.recordOutput("Shooter/GoalIntent", currentGoal.intent().toString());

	}
}
