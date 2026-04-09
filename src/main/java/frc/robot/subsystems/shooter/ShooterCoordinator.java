package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Superstructure;
import frc.robot.subsystems.shooter.aiming.Aiming;

public class ShooterCoordinator extends SubsystemBase{

	public static ShooterGoal currentGoal;

    public enum ShooterIntent{

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

	public enum ShooterMode{

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

    public ShooterCoordinator() {
		currentGoal = new ShooterGoal(ShooterMode.IDLE, ShooterIntent.INACTIVE);
    }

	/**
	 * @return whether or not the operator wants to be actively firing
	 */
	public boolean operatorWantsToFire(){
		return RobotContainer.operatorWantsToFireTrigger.getAsBoolean();
	}

	/**
	 * @return whether or not the operator wants to track the hub
	 */
	public boolean operatorWantsToTrackHub(){
		return RobotContainer.operatorWantsToTrackHubTrigger.getAsBoolean();
	}

	/**
	 * @return whether or not the operator wants to track a ferry point
	 */
	public boolean operatorWantsToTrackFerryPoint(){
		return RobotContainer.operatorWantsToTrackFerryPointTrigger.getAsBoolean();
	}

    public ShooterGoal calculateGoal(
		boolean isInScoringZone,
		boolean isInLeftFerryZone,
		boolean isInRightFerryZone,
		boolean isInTrenchZone,
		boolean operatorWantsToTrackHub,
		boolean operatorWantsToTrackFerryPoint,
		boolean operatorWantsToFire
	) {

		if(isInTrenchZone) {
			// we keep the mode to remain tracking the desired goal
			// we force the intent to ARMED which lowers the hood and stops all firing
			return new ShooterGoal(currentGoal.mode, ShooterIntent.ARMED);
		}

		// if we're in auto, we force scoring but only FIRE while in the alliance zone
		if(RobotContainer.shouldShootAuto) {
			if(isInScoringZone) {
				return new ShooterGoal(ShooterMode.SCORING, ShooterIntent.FIRING);
			} else {
				return new ShooterGoal(ShooterMode.SCORING, ShooterIntent.ARMED);
			}
		}

		// when the operator wants to track the hub
		if(operatorWantsToTrackHub) {
			if(operatorWantsToFire && isInScoringZone) {
				return new ShooterGoal(ShooterMode.SCORING, ShooterIntent.FIRING);
			} else {
				return new ShooterGoal(ShooterMode.SCORING, ShooterIntent.ARMED);
			}
		}

		// when the operator wants to ferry
		if(operatorWantsToTrackFerryPoint){
			if(isInLeftFerryZone) {
				if(operatorWantsToFire) {
					return new ShooterGoal(ShooterMode.FERRYING_LEFT, ShooterIntent.FIRING);
				} else {
					return new ShooterGoal(ShooterMode.FERRYING_LEFT, ShooterIntent.ARMED);
				}
			} else if(isInRightFerryZone) {
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
			Superstructure.isInScoringZone(),
			Superstructure.isInLeftFerryZone(),
			Superstructure.isInRightFerryZone(),
			Superstructure.isInTrenchDangerZone(),
			operatorWantsToTrackHub(),
			operatorWantsToTrackFerryPoint(),
			operatorWantsToFire()
		);

		Logger.recordOutput("Shooter/GoalMode", currentGoal.mode().toString());
		Logger.recordOutput("Shooter/GoalIntent", currentGoal.intent().toString());
	}
}
