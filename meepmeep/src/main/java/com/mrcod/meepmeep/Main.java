package com.mrcod.meepmeep;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.ColorScheme;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.roadrunner.Constraints;
import com.noahbres.meepmeep.roadrunner.DriveTrainType;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySequence;
import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySequenceBuilder;

import static com.mrcod.meepmeep.MeepMeepHelper.inchesToCoordinate;

public class Main {
    public static void main(String[] args) {
        System.setProperty("sun.java2d.opengl", "true");

        MeepMeep meep = new MeepMeep(800);
        meep.setAxesInterval(10);
        meep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_KAI_DARK);
        ColorScheme scheme = new ColorSchemeBlueDark();
        meep.setTheme(scheme);
        meep.setBackgroundAlpha(1);

        meep.addEntity(closeBase(meep));

        meep.start();
    }

    public static RoadRunnerBotEntity closeBase(MeepMeep meep) {
        final Pose2d startPose = new Pose2d(-35, 70 - inchesToCoordinate(9D),
                Math.toRadians(90));

        RoadRunnerBotEntity roadRunnerBot = new RoadRunnerBotEntity(meep, DriveConstants.CONSTRAINTS,
                inchesToCoordinate(18D), inchesToCoordinate(18D), startPose, new ColorSchemeBlueDark(),
                0.5D, DriveTrainType.MECANUM, false);

        TrajectorySequenceBuilder builder = new TrajectorySequenceBuilder(startPose,
                new MecanumVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.TRACK_WIDTH),
                new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL),
                DriveConstants.MAX_ANG_VEL, DriveConstants.MAX_ANG_ACCEL);

        builder.strafeTo(new Vector2d(-60, 58));
        for (int i = 0; i < 3; i++) {
            builder.strafeTo(new Vector2d(-57, 13));
            builder.waitSeconds(0.5);
            builder.strafeTo(new Vector2d(-22, 13));
            builder.waitSeconds(0.5);
        }

        roadRunnerBot.followTrajectorySequence(builder.build());

        return roadRunnerBot;
    }
}

