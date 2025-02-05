package com.mrcod.meepmeep;

import static com.mrcod.meepmeep.CoordinateUtil.RRToGridCoordinate;
import static com.mrcod.meepmeep.CoordinateUtil.trajectoryTo;
import static com.mrcod.meepmeep.MirroringUtil.cMirrorY;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.mrcod.meepmeep.entity.helper.LineEntity;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.ColorScheme;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueLight;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedLight;
import com.noahbres.meepmeep.roadrunner.DriveTrainType;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySequenceBuilder;

import java.awt.event.KeyEvent;
import java.awt.event.KeyListener;


public class Main {
    public static void main(String[] args) {
        System.setProperty("sun.java2d.opengl", "true");

        MeepMeep meep = new MeepMeep(700);
        meep.setAxesInterval(10);
        meep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_KAI_DARK);
        ColorScheme scheme = new ColorSchemeBlueDark();
        meep.setTheme(scheme);
        meep.setBackgroundAlpha(1);

        double centers = -60;
        for (double i = centers; i < 72; i += 24) {
            meep.addEntity(new LineEntity(new Vector2d(i, 72), new Vector2d(i, -72), meep));
        }

        for (double i = centers; i < 72; i += 24) {
            meep.addEntity(new LineEntity(new Vector2d(72, i), new Vector2d(-72, i), meep));
        }

        meep.addEntity(closeBase(meep, false));
        meep.addEntity(closeBase(meep, true));
//        meep.addEntity(patrickOpMODe(meep,false));
//        meep.addEntity(patrickOpMODe(meep,true));
//        RoadRunnerBotEntity bot = coordinateBot(meep);
//        meep.addEntity(bot);
//        bot.setLooping(false);


        meep.start();
    }

    public static RoadRunnerBotEntity closeBase(MeepMeep meep, boolean mirror) {
        final Pose2d startPose = cMirrorY(new Pose2d(-32, 63,
                Math.toRadians(90)), mirror);

        RoadRunnerBotEntity roadRunnerBot = new RoadRunnerBotEntity(meep, DriveConstants.CONSTRAINTS,
                16, 16, startPose, mirror ? new ColorSchemeRedDark() : new ColorSchemeBlueDark(),
                0.5D, DriveTrainType.MECANUM, false);

        TrajectorySequenceBuilder builder = new TrajectorySequenceBuilder(startPose,
                new MecanumVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.TRACK_WIDTH),
                new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL),
                DriveConstants.MAX_ANG_VEL, DriveConstants.MAX_ANG_ACCEL);

        builder.strafeTo(cMirrorY(new Vector2d(-35, 63), mirror));
        builder.strafeTo(cMirrorY(new Vector2d(-35, 12), mirror));
        builder.strafeTo(cMirrorY(new Vector2d(-22, 12), mirror));
        builder.waitSeconds(0.5);

        for (int i = 1; i <= 5; i++) {
            builder.strafeTo(cMirrorY(new Vector2d(-57, 12), mirror));
            builder.waitSeconds(0.5);
            builder.strafeTo(cMirrorY(new Vector2d(-22, 12), mirror));
            builder.waitSeconds(0.5);
        }

        roadRunnerBot.followTrajectorySequence(builder.build());

        return roadRunnerBot;
    }

    public static RoadRunnerBotEntity coordinateBot(MeepMeep meep) {
        final Pose2d startPose = new Pose2d(10, 63,
                Math.toRadians(0));

        RoadRunnerBotEntity roadRunnerBot = new RoadRunnerBotEntity(meep, DriveConstants.CONSTRAINTS,
                18, 18, startPose, new ColorSchemeBlueDark(),
                0.5D, DriveTrainType.MECANUM, false);

        meep.getCanvas().addKeyListener(new BotKeyListener(roadRunnerBot));
        meep.getCanvas().setFocusable(true);
        meep.getCanvas().requestFocus();

        TrajectorySequenceBuilder builder = new TrajectorySequenceBuilder(startPose,
                new MecanumVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.TRACK_WIDTH),
                new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL),
                DriveConstants.MAX_ANG_VEL, DriveConstants.MAX_ANG_ACCEL);

        builder.strafeTo(new Vector2d(-60, 58));

        roadRunnerBot.followTrajectorySequence(builder.build());
        roadRunnerBot.setLooping(false);
        return roadRunnerBot;
    }

    public static RoadRunnerBotEntity patrickOpMODe(MeepMeep meep, boolean mirror) {
        final Pose2d startPose = cMirrorY(new Pose2d(-40,40), mirror);

        RoadRunnerBotEntity roadRunnerBot = new RoadRunnerBotEntity(meep, DriveConstants.CONSTRAINTS,
                18, 18, startPose, mirror ? new ColorSchemeRedLight() : new ColorSchemeBlueLight(),
                0.2, DriveTrainType.MECANUM, false);

        TrajectorySequenceBuilder builder = new TrajectorySequenceBuilder(startPose,
                new MecanumVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.TRACK_WIDTH),
                new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL),
                DriveConstants.MAX_ANG_VEL, DriveConstants.MAX_ANG_ACCEL);

        builder.strafeTo(cMirrorY(new Vector2d(40, 30), mirror));

        roadRunnerBot.followTrajectorySequence(builder.build());

        return roadRunnerBot;
    }

    static class BotKeyListener implements KeyListener {
        public final RoadRunnerBotEntity bot;
        public CoordinateUtil.Vector2i previousGridCoordinate;
        public CoordinateUtil.Vector2i choice;
        public double heading;

        public BotKeyListener(RoadRunnerBotEntity bot) {
            this.bot = bot;
            previousGridCoordinate = RRToGridCoordinate(bot.getPose().vec());
            choice = previousGridCoordinate;
            heading = bot.getPose().getHeading();
            System.out.println("Key listener initialized");
        }

        @Override
        public void keyTyped(KeyEvent keyEvent) {
        }

        @Override
        public void keyPressed(KeyEvent keyEvent) {
            int chosenX = -1;
            int chosenY = -1;

            switch (keyEvent.getKeyCode()) {
                case KeyEvent.VK_1:
                    chosenX = 0;
                    break;
                case KeyEvent.VK_2:
                    chosenX = 1;
                    break;
                case KeyEvent.VK_3:
                    chosenX = 2;
                    break;
                case KeyEvent.VK_4:
                    chosenX = 3;
                    break;
                case KeyEvent.VK_5:
                    chosenX = 4;
                    break;
                case KeyEvent.VK_6:
                    chosenX = 5;
                    break;
                case KeyEvent.VK_Q:
                    chosenY = 0;
                    break;
                case KeyEvent.VK_W:
                    chosenY = 1;
                    break;
                case KeyEvent.VK_E:
                    chosenY = 2;
                    break;
                case KeyEvent.VK_R:
                    chosenY = 3;
                    break;
                case KeyEvent.VK_T:
                    chosenY = 4;
                    break;
                case KeyEvent.VK_Y:
                    chosenY = 5;
                    break;
                case KeyEvent.VK_A:
                    heading -= Math.PI / 2;
                    if (heading <= 0) {
                        heading += Math.PI;
                    }
                    break;
                case KeyEvent.VK_D:
                    heading += Math.PI / 2;
                    if (heading >= Math.PI * 2) {
                        heading -= Math.PI;
                    }
                    break;
            }

            if (chosenX == -1) {
                chosenX = choice.x;
            }

            if (chosenY == -1) {
                chosenY = choice.y;
            }

            choice = new CoordinateUtil.Vector2i(chosenX, chosenY);

            if (keyEvent.getKeyCode() == KeyEvent.VK_ENTER) {
                Pose2d startPose = bot.getPose();

                bot.followTrajectorySequence(trajectoryTo(startPose, choice, heading));
                bot.setTrajectoryProgressSeconds(0);
                bot.setLooping(false);

                previousGridCoordinate = choice;
            }
        }

        @Override
        public void keyReleased(KeyEvent keyEvent) {
        }
    }
}

