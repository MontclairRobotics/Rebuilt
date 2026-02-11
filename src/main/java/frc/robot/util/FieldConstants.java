// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.util;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import edu.wpi.first.units.measure.Distance;
import frc.robot.constants.Constants;

/**
 * credit to team 6328 for this wonderful class of tag-based constants.
 */
public class FieldConstants {
	public static final FieldType FIELD_TYPE = FieldType.WELDED;

	// AprilTag related constants
	public static final int APRIL_TAG_COUNT = AprilTagLayoutType.OFFICIAL.getLayout().getTags().size();
	public static final Distance APRIL_TAG_WIDTH = Inches.of(6.5);
	public static final AprilTagLayoutType DEFAULT_APRIL_TAG_LAYOUT_TYPE = AprilTagLayoutType.OFFICIAL;

	// Field dimensions
	public static final Distance FIELD_LENGTH = Meters.of(AprilTagLayoutType.OFFICIAL.getLayout().getFieldLength());
	public static final Distance FIELD_WIDTH = Meters.of(AprilTagLayoutType.OFFICIAL.getLayout().getFieldWidth());

	public static class Zones {

		// space (along x axis) in front and behind the center of the TRENCH to count for the TRENCH zone
        public static final Distance TRENCH_ZONE_EXTENSION = Inches.of(90);

		public static final Distance TRENCH_DANGER_ZONE_EXTENSION = Inches.of(40);

		// space (along x axis) in front and behind the center of the BUMP to count for the BUMP zone
        private static final Distance BUMP_ZONE_EXTENSION = Inches.of(80);

		private static final Distance FUDGE_FACTOR = Constants.BUMPER_WIDTH.div(2);

        public static final Translation2d[][] TRENCH_ZONES = {

			// near right trench
            new Translation2d[] {
                new Translation2d(LinesVertical.HUB_CENTER.minus(TRENCH_ZONE_EXTENSION), Meters.zero()),
                new Translation2d(LinesVertical.HUB_CENTER.plus(TRENCH_ZONE_EXTENSION), RightTrench.WIDTH)
            },

			// near left trench
            new Translation2d[] {
                new Translation2d(LinesVertical.HUB_CENTER.minus(TRENCH_ZONE_EXTENSION), FIELD_WIDTH.minus(LeftTrench.WIDTH)),
                new Translation2d(LinesVertical.HUB_CENTER.plus(TRENCH_ZONE_EXTENSION), FIELD_WIDTH)
            },

			// far right trench
            new Translation2d[] {
                new Translation2d(LinesVertical.OPP_HUB_CENTER.minus(TRENCH_ZONE_EXTENSION), Meters.zero()),
                new Translation2d(LinesVertical.OPP_HUB_CENTER.plus(TRENCH_ZONE_EXTENSION), RightTrench.WIDTH)
            },

			// far left trench
            new Translation2d[] {
                new Translation2d(LinesVertical.OPP_HUB_CENTER.minus(TRENCH_ZONE_EXTENSION), FIELD_WIDTH.minus(LeftTrench.WIDTH)),
                new Translation2d(LinesVertical.OPP_HUB_CENTER.plus(TRENCH_ZONE_EXTENSION), FIELD_WIDTH)
            }
        };

		public static final Translation2d[][] TRENCH_DANGER_ZONES = {

			// near right trench
            new Translation2d[] {
                new Translation2d(LinesVertical.HUB_CENTER.minus(TRENCH_DANGER_ZONE_EXTENSION), Meters.zero()),
                new Translation2d(LinesVertical.HUB_CENTER.plus(TRENCH_DANGER_ZONE_EXTENSION), RightTrench.WIDTH)
            },

			// near left trench
            new Translation2d[] {
                new Translation2d(LinesVertical.HUB_CENTER.minus(TRENCH_DANGER_ZONE_EXTENSION), FIELD_WIDTH.minus(LeftTrench.WIDTH)),
                new Translation2d(LinesVertical.HUB_CENTER.plus(TRENCH_DANGER_ZONE_EXTENSION), FIELD_WIDTH)
            },

			// far right trench
            new Translation2d[] {
                new Translation2d(LinesVertical.OPP_HUB_CENTER.minus(TRENCH_DANGER_ZONE_EXTENSION), Meters.zero()),
                new Translation2d(LinesVertical.OPP_HUB_CENTER.plus(TRENCH_DANGER_ZONE_EXTENSION), RightTrench.WIDTH)
            },

			// far left trench
            new Translation2d[] {
                new Translation2d(LinesVertical.OPP_HUB_CENTER.minus(TRENCH_DANGER_ZONE_EXTENSION), FIELD_WIDTH.minus(LeftTrench.WIDTH)),
                new Translation2d(LinesVertical.OPP_HUB_CENTER.plus(TRENCH_DANGER_ZONE_EXTENSION), FIELD_WIDTH)
            }
        };

        public static final Translation2d[][] BUMP_ZONES = {

			// near right bump
            new Translation2d[] {
                new Translation2d(LinesVertical.HUB_CENTER.minus(BUMP_ZONE_EXTENSION), RightBump.NEAR_RIGHT_CORNER.getMeasureY().plus(FUDGE_FACTOR)),
                new Translation2d(LinesVertical.HUB_CENTER.plus(BUMP_ZONE_EXTENSION), RightBump.FAR_LEFT_CORNER.getMeasureY().minus(FUDGE_FACTOR))
            },

			// near left bump
            new Translation2d[] {
                new Translation2d(LinesVertical.HUB_CENTER.minus(BUMP_ZONE_EXTENSION), LeftBump.NEAR_RIGHT_CORNER.getMeasureY().plus(FUDGE_FACTOR)),
                new Translation2d(LinesVertical.HUB_CENTER.plus(BUMP_ZONE_EXTENSION), LeftBump.FAR_LEFT_CORNER.getMeasureY().minus(FUDGE_FACTOR))
            },

			// far right bump
            new Translation2d[] {
                new Translation2d(LinesVertical.OPP_HUB_CENTER.minus(BUMP_ZONE_EXTENSION), RightBump.OPP_NEAR_RIGHT_CORNER.getMeasureY().plus(FUDGE_FACTOR)),
                new Translation2d(LinesVertical.OPP_HUB_CENTER.plus(BUMP_ZONE_EXTENSION), RightBump.OPP_FAR_LEFT_CORNER.getMeasureY().minus(FUDGE_FACTOR))
            },

			// far left bump
            new Translation2d[] {
                new Translation2d(LinesVertical.OPP_HUB_CENTER.minus(BUMP_ZONE_EXTENSION), LeftBump.OPP_NEAR_RIGHT_CORNER.getMeasureY().plus(FUDGE_FACTOR)),
                new Translation2d(LinesVertical.OPP_HUB_CENTER.plus(BUMP_ZONE_EXTENSION), LeftBump.OPP_FAR_LEFT_CORNER.getMeasureY().minus(FUDGE_FACTOR))
            }
        };

	}

	/**
	 * Officially defined and relevant vertical lines found on the field (defined by X-axis offset)
	 */
	public static class LinesVertical {
		public static final Distance CENTER = FIELD_LENGTH.div(2.0);
		public static final Distance STARTING = AprilTagLayoutType.OFFICIAL.getLayout().getTagPose(26).get().getMeasureX();
		public static final Distance ALLIANCE_ZONE = STARTING;
		public static final Distance HUB_CENTER = AprilTagLayoutType.OFFICIAL.getLayout().getTagPose(26).get().getMeasureX().plus(Hub.WIDTH.div(2.0));
		public static final Distance NEUTRAL_ZONE_NEAR = CENTER.minus(Inches.of(120));
		public static final Distance NEUTRAL_ZONE_FAR = CENTER.plus(Inches.of(120));
		public static final Distance OPP_HUB_CENTER = AprilTagLayoutType.OFFICIAL.getLayout().getTagPose(4).get().getMeasureX().plus(Hub.WIDTH.div(2.0));
		public static final Distance OPP_ALLIANCE_ZONE = AprilTagLayoutType.OFFICIAL.getLayout().getTagPose(10).get().getMeasureX();
        public static double neutralZoneFar;
	}

	/**
	 * Officially defined and relevant horizontal lines found on the field (defined by Y-axis offset)
	 *
	 * <p>NOTE: The field element start and end are always left to right from the perspective of the
	 * alliance station
	 */
	public static class LinesHorizontal {

		public static final Distance CENTER = FIELD_WIDTH.div(2.0);

		// Right of hub
		public static final Distance RIGHT_BUMP_START = Hub.NEAR_RIGHT_CORNER.getMeasureY();
		public static final Distance RIGHT_BUMP_END = RIGHT_BUMP_START.minus(RightBump.WIDTH);
		public static final Distance RIGHT_TRENCH_OPEN_START = RIGHT_BUMP_END.minus(Inches.of(12));
		public static final Distance RIGHT_TRENCH_OPEN_END = Meters.zero();

		// Left of hub
		public static final Distance LEFT_BUMP_END = Hub.NEAR_LEFT_CORNER.getMeasureY();
		public static final Distance LEFT_BUMP_START = LEFT_BUMP_END.plus(LeftBump.WIDTH);
		public static final Distance LEFT_TRENCH_OPEN_END = LEFT_BUMP_START.plus(Inches.of(12.0));
		public static final Distance LEFT_TRENCH_OPEN_START = FIELD_WIDTH;
	}

	/** Hub related constants */
	public static class Hub {

		// Dimensions
		public static final Distance WIDTH = Inches.of(47);
		public static final Distance HEIGHT =
			Inches.of(72.0); // includes the catcher at the top
		public static final Distance INNER_WIDTH = Inches.of(41.7);
		public static final Distance INNER_HEIGHT = Inches.of(56.5);

		// Relevant reference points on alliance side
		public static final Translation3d TOP_CENTER_POINT =
			new Translation3d(
				AprilTagLayoutType.OFFICIAL.getLayout().getTagPose(26).get().getMeasureX().plus(WIDTH.div(2.0)),
				FIELD_WIDTH.div(2.0),
				HEIGHT);
		public static final Translation3d INNER_CENTER_POINT =
			new Translation3d(
				AprilTagLayoutType.OFFICIAL.getLayout().getTagPose(26).get().getMeasureX().plus(WIDTH.div(2.0)),
				FIELD_WIDTH.div(2.0),
				INNER_HEIGHT);

		public static final Translation2d HUB_LOCATION = new Translation2d(4.6256, 4.034);

		public static final Translation2d NEAR_LEFT_CORNER =
			new Translation2d(TOP_CENTER_POINT.getMeasureX().minus(WIDTH.div(2.0)), FIELD_WIDTH.div(2.0).plus(WIDTH.div(2.0)));
		public static final Translation2d NEAR_RIGHT_CORNER =
			new Translation2d(TOP_CENTER_POINT.getMeasureX().minus(WIDTH.div(2.0)), FIELD_WIDTH.div(2.0).minus(WIDTH.div(2.0)));
		public static final Translation2d FAR_LEFT_CORNER =
			new Translation2d(TOP_CENTER_POINT.getMeasureX().plus(WIDTH.div(2.0)), FIELD_WIDTH.div(2.0).plus(WIDTH.div(2.0)));
		public static final Translation2d FAR_RIGHT_CORNER =
			new Translation2d(TOP_CENTER_POINT.getMeasureX().plus(WIDTH.div(2.0)), FIELD_WIDTH.div(2.0).minus(WIDTH.div(2.0)));

		// Relevant reference points on the opposite side
		public static final Translation3d OPP_TOP_CENTER_POINT =
			new Translation3d(
				AprilTagLayoutType.OFFICIAL.getLayout().getTagPose(4).get().getMeasureX().plus(WIDTH.div(2.0)),
				FIELD_WIDTH.div(2.0),
				HEIGHT);
		public static final Translation2d OPP_NEAR_LEFT_CORNER =
			new Translation2d(OPP_TOP_CENTER_POINT.getMeasureX().minus(WIDTH.div(2.0)), FIELD_WIDTH.div(2.0).plus(WIDTH.div(2.0)));
		public static final Translation2d OPP_NEAR_RIGHT_CORNER =
			new Translation2d(OPP_TOP_CENTER_POINT.getMeasureX().minus(WIDTH.div(2.0)), FIELD_WIDTH.div(2.0).minus(WIDTH.div(2.0)));
		public static final Translation2d OPP_FAR_LEFT_CORNER =
			new Translation2d(OPP_TOP_CENTER_POINT.getMeasureX().plus(WIDTH.div(2.0)), FIELD_WIDTH.div(2.0).plus(WIDTH.div(2.0)));
		public static final Translation2d OPP_FAR_RIGHT_CORNER =
			new Translation2d(OPP_TOP_CENTER_POINT.getMeasureX().plus(WIDTH.div(2.0)), FIELD_WIDTH.div(2.0).minus(WIDTH.div(2.0)));

		// Hub faces
		public static final Pose2d NEAR_FACE =
			AprilTagLayoutType.OFFICIAL.getLayout().getTagPose(26).get().toPose2d();
		public static final Pose2d FAR_FACE =
			AprilTagLayoutType.OFFICIAL.getLayout().getTagPose(20).get().toPose2d();
		public static final Pose2d RIGHT_FACE =
			AprilTagLayoutType.OFFICIAL.getLayout().getTagPose(18).get().toPose2d();
		public static final Pose2d LEFT_FACE =
			AprilTagLayoutType.OFFICIAL.getLayout().getTagPose(21).get().toPose2d();
	}

	/** Left Bump related constants */
	public static class LeftBump {

		// Dimensions
		public static final Distance WIDTH = Inches.of(73.0);
		public static final Distance HEIGHT = Inches.of(6.513);
		public static final Distance DEPTH = Inches.of(44.4);

		// Relevant reference points on alliance side
		public static final Translation2d NEAR_LEFT_CORNER =
			new Translation2d(LinesVertical.HUB_CENTER.minus(WIDTH.div(2.0)), Inches.of(255));
		public static final Translation2d NEAR_RIGHT_CORNER = Hub.NEAR_LEFT_CORNER;
		public static final Translation2d FAR_LEFT_CORNER =
			new Translation2d(LinesVertical.HUB_CENTER.plus(WIDTH.div(2.0)), Inches.of(255));
		public static final Translation2d FAR_RIGHT_CORNER = Hub.FAR_LEFT_CORNER;

		// Relevant reference points on opposing side
		public static final Translation2d OPP_NEAR_LEFT_CORNER =
			new Translation2d(LinesVertical.HUB_CENTER.minus(WIDTH.div(2.0)), Inches.of(255));
		public static final Translation2d OPP_NEAR_RIGHT_CORNER = Hub.OPP_NEAR_LEFT_CORNER;
		public static final Translation2d OPP_FAR_LEFT_CORNER =
			new Translation2d(LinesVertical.HUB_CENTER.plus(WIDTH.div(2.0)), Inches.of(255));
		public static final Translation2d OPP_FAR_RIGHT_CORNER = Hub.OPP_FAR_LEFT_CORNER;
	}

	/** Right Bump related constants */
	public static class RightBump {
		// Dimensions
		public static final Distance WIDTH = Inches.of(73.0);
		public static final Distance HEIGHT = Inches.of(6.513);
		public static final Distance DEPTH = Inches.of(44.4);

		// Relevant reference points on alliance side
		public static final Translation2d NEAR_RIGHT_CORNER =
			new Translation2d(LinesVertical.HUB_CENTER.minus(WIDTH.div(2.0)), Inches.of(62.7));
		public static final Translation2d NEAR_LEFT_CORNER = Hub.NEAR_RIGHT_CORNER;
		public static final Translation2d FAR_RIGHT_CORNER =
			new Translation2d(LinesVertical.HUB_CENTER.plus(WIDTH.div(2.0)), Inches.of(62.7));
		public static final Translation2d FAR_LEFT_CORNER = Hub.FAR_RIGHT_CORNER;

		// Relevant reference points on opposing side
		public static final Translation2d OPP_NEAR_RIGHT_CORNER =
			new Translation2d(LinesVertical.HUB_CENTER.minus(WIDTH.div(2.0)), Inches.of(62.7));
		public static final Translation2d OPP_NEAR_LEFT_CORNER = Hub.OPP_NEAR_RIGHT_CORNER;
		public static final Translation2d OPP_FAR_RIGHT_CORNER =
			new Translation2d(LinesVertical.HUB_CENTER.plus(WIDTH.div(2.0)), Inches.of(62.7));
		public static final Translation2d OPP_FAR_LEFT_CORNER = Hub.OPP_FAR_RIGHT_CORNER;
	}

	/** Left Trench related constants */
	public static class LeftTrench {
		// Dimensions
		public static final Distance WIDTH = Inches.of(65.65);
		public static final Distance DEPTH = Inches.of(47.0);
		public static final Distance HEIGHT = Inches.of(40.25);
		public static final Distance OPENING_WIDTH = Inches.of(50.34);
		public static final Distance OPENING_HEIGHT = Inches.of(22.25);

		// Relevant reference points on alliance side
		public static final Translation3d OPENING_TOP_LEFT =
			new Translation3d(LinesVertical.HUB_CENTER, FIELD_WIDTH, OPENING_HEIGHT);
		public static final Translation3d OPENING_TOP_RIGHT =
			new Translation3d(LinesVertical.HUB_CENTER, FIELD_WIDTH.minus(OPENING_WIDTH), OPENING_HEIGHT);

		// Relevant reference points on opposing side
		public static final Translation3d OPP_OPENING_TOP_LEFT =
			new Translation3d(LinesVertical.OPP_HUB_CENTER, FIELD_WIDTH, OPENING_HEIGHT);
		public static final Translation3d OPP_OPENING_TOP_RIGHT =
			new Translation3d(LinesVertical.OPP_HUB_CENTER, FIELD_WIDTH.minus(OPENING_WIDTH), OPENING_HEIGHT);
	}

	public static class RightTrench {

		// Dimensions
		public static final Distance WIDTH = Inches.of(65.65);
		public static final Distance DEPTH = Inches.of(47.0);
		public static final Distance HEIGHT = Inches.of(40.25);
		public static final Distance OPENING_WIDTH = Inches.of(50.34);
		public static final Distance OPENING_HEIGHT = Inches.of(22.25);

		// Relevant reference points on alliance side
		public static final Translation3d OPENING_TOP_LEFT =
			new Translation3d(LinesVertical.HUB_CENTER, OPENING_WIDTH, OPENING_HEIGHT);
		public static final Translation3d OPENING_TOP_RIGHT =
			new Translation3d(LinesVertical.HUB_CENTER, Meters.zero(), OPENING_HEIGHT);

		// Relevant reference points on opposing side
		public static final Translation3d OPP_OPENING_TOP_LEFT =
			new Translation3d(LinesVertical.OPP_HUB_CENTER, OPENING_WIDTH, OPENING_HEIGHT);
		public static final Translation3d OPP_OPENING_TOP_RIGHT =
			new Translation3d(LinesVertical.OPP_HUB_CENTER, Meters.zero(), OPENING_HEIGHT);
	}

	/** Tower related constants */
	public static class Tower {
		// Dimensions
		public static final Distance WIDTH = Inches.of(49.25);
		public static final Distance DEPTH = Inches.of(45.0);
		public static final Distance HEIGHT = Inches.of(78.25);
		public static final Distance INNER_OPENING_WIDTH = Inches.of(32.250);
		public static final Distance FRONT_FACE_X = Inches.of(43.51);

		public static final Distance UPRIGHT_HEIGHT = Inches.of(72.1);

		// Rung heights from the floor
		public static final Distance LOW_RUNG_HEIGHT = Inches.of(27.0);
		public static final Distance MID_RUNG_HEIGHT = Inches.of(45.0);
		public static final Distance HIGH_RUNG_HEIGHT = Inches.of(63.0);

		// Relevant reference points on alliance side
		public static final Translation2d CENTER_POINT =
			new Translation2d(
				FRONT_FACE_X,
				AprilTagLayoutType.OFFICIAL.getLayout().getTagPose(31).get().getMeasureY());
		public static final Translation2d LEFT_UPRIGHT =
			new Translation2d(
				FRONT_FACE_X,
				AprilTagLayoutType.OFFICIAL.getLayout().getTagPose(31).get().getMeasureY().plus(INNER_OPENING_WIDTH.div(2.0)).plus(Inches.of(0.75))
			);
		public static final Translation2d RIGHT_UPRIGHT =
			new Translation2d(
				FRONT_FACE_X,
				AprilTagLayoutType.OFFICIAL.getLayout().getTagPose(31).get().getMeasureY().minus(INNER_OPENING_WIDTH.div(2.0)).minus(Inches.of(0.75))
			);

		// Relevant reference points on opposing side
		public static final Translation2d OPP_CENTER_POINT =
			new Translation2d(
				FIELD_LENGTH.minus(FRONT_FACE_X),
				AprilTagLayoutType.OFFICIAL.getLayout().getTagPose(15).get().getMeasureY()
			);
		public static final Translation2d OPP_LEFT_UPRIGHT =
			new Translation2d(
				FIELD_LENGTH.minus(FRONT_FACE_X),
				AprilTagLayoutType.OFFICIAL.getLayout().getTagPose(15).get().getMeasureY().plus(INNER_OPENING_WIDTH.div(2.0)).plus(Inches.of(0.75))
			);
		public static final Translation2d OPP_RIGHT_UPRIGHT =
			new Translation2d(
				FIELD_LENGTH.minus(FRONT_FACE_X),
				AprilTagLayoutType.OFFICIAL.getLayout().getTagPose(15).get().getMeasureY().minus(INNER_OPENING_WIDTH.div(2.0)).minus(Inches.of(0.75))
			);
	}

	public static class Depot {
		// Dimensions
		public static final Distance WIDTH = Inches.of(42.0);
		public static final Distance DEPTH = Inches.of(27.0);
		public static final Distance HEIGHT = Inches.of(1.125);
		public static final Distance DISTANCE_FROM_CENTER_Y = Inches.of(75.93);

		// Relevant reference points on alliance side
		public static final Translation3d DEPOT_CORNER =
			new Translation3d(DEPTH, FIELD_WIDTH.div(2.0).plus(DISTANCE_FROM_CENTER_Y), HEIGHT);
		public static final Translation3d LEFT_CORNER =
			new Translation3d(DEPTH, FIELD_WIDTH.div(2.0).plus(DISTANCE_FROM_CENTER_Y).plus(WIDTH.div(2.0)), HEIGHT);
		public static final Translation3d RIGHT_CORNER =
			new Translation3d(DEPTH, FIELD_WIDTH.div(2.0).plus(DISTANCE_FROM_CENTER_Y).minus(WIDTH.div(2.0)), HEIGHT);
	}

	public static class Outpost {
		// Dimensions
		public static final Distance WIDTH = Inches.of(31.8);
		public static final Distance OPENING_DISTANCE_FROM_FLOOR = Inches.of(28.1);
		public static final Distance HEIGHT = Inches.of(7);

		// Relevant reference points on alliance side
		public static final Translation2d CENTER_POINT =
			new Translation2d(Meters.zero(), AprilTagLayoutType.OFFICIAL.getLayout().getTagPose(29).get().getMeasureY());
	}

	public static class ferryWaypoints{
		// TODO: find these values
		public static final Translation2d LEFT_FERRYING_POINT = new Translation2d(3, 5);
		public static final Translation2d RIGHT_FERRYING_POINT = new Translation2d(3, 3);
	
		public static final double LEFT_FERRYING_HEIGHT = 0.0;
		public static final double RIGHT_FERRYING_HEIGHT = 0.0;
	}

	public enum FieldType {
		ANDYMARK("andymark"),
		WELDED("welded");

		private final String jsonFolder;

		FieldType(String jsonFolder) {
			this.jsonFolder = jsonFolder;
		}
	}

	public enum AprilTagLayoutType {
		OFFICIAL("2026-official"),
		NONE("2026-none");

		private final String name;
		private volatile AprilTagFieldLayout layout;
		private volatile String layoutString;

		AprilTagLayoutType(String name) {
			this.name = name;
		}

		public AprilTagFieldLayout getLayout() {
			return AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);
		}

		public String getLayoutString() {
			if (layoutString == null) {
				getLayout();
			}
			return layoutString;
		}
	}
}
