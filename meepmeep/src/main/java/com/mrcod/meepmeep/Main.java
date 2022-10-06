package com.mrcod.meepmeep;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.mrcod.meepmeep.entity.helper.LineEntity;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.ColorScheme;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.roadrunner.DriveTrainType;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySequenceBuilder;

import java.awt.event.KeyEvent;
import java.awt.event.KeyListener;

public class Main {
    public static void main(String[] args) {
        System.setProperty("sun.java2d.opengl", "true");

        MeepMeep meep = new MeepMeep(800);
        meep.setAxesInterval(10);
        meep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL);
        ColorScheme scheme = new ColorSchemeBlueDark();
        meep.setTheme(scheme);
        meep.setBackgroundAlpha(1);

        double centers = -60;
        for (double i = centers; i < 72; i += 24) {
            meep.addEntity(new LineEntity(new Vector2d(i,72), new Vector2d(i, -72), meep));
        }

        for (double i = centers; i < 72; i += 24) {
            meep.addEntity(new LineEntity(new Vector2d(72,i), new Vector2d(-72, i), meep));
        }

//        meep.addEntity(closeBase(meep));
        meep.addEntity(coordinateBot(meep));

        meep.start();
    }

    public static RoadRunnerBotEntity closeBase(MeepMeep meep) {
        final Pose2d startPose = new Pose2d(-35, 63,
                Math.toRadians(90));

        RoadRunnerBotEntity roadRunnerBot = new RoadRunnerBotEntity(meep, DriveConstants.CONSTRAINTS,
                18, 18, startPose, new ColorSchemeBlueDark(),
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

    public static RoadRunnerBotEntity coordinateBot(MeepMeep meep) {
        final Pose2d startPose = new Pose2d(10, 63,
                Math.toRadians(90));

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

        return roadRunnerBot;
    }

    static class Vector2i {
        public final int x;
        public final int y;

        public Vector2i(int x, int y) {
            this.x = x;
            this.y = y;
        }
    }


    static class BotKeyListener implements KeyListener {
        public final RoadRunnerBotEntity bot;
        public Vector2i previousGridCoordinate;

        public BotKeyListener(RoadRunnerBotEntity bot) {
            this.bot = bot;
            previousGridCoordinate = RRToGridCoordinate(bot.getPose().vec());
            System.out.println("Key listener initialized");
        }

        @Override
        public void keyTyped(KeyEvent keyEvent) {
        }

        @Override
        public void keyPressed(KeyEvent keyEvent) {
            previousGridCoordinate = RRToGridCoordinate(bot.getPose().vec());

            int chosenX = -1;
            int chosenY = -1;

            if (keyEvent.getKeyCode() == KeyEvent.VK_0) {
                chosenX = 0;
            } else if (keyEvent.getKeyCode() == KeyEvent.VK_1) {
                chosenX = 1;
            } else if (keyEvent.getKeyCode() == KeyEvent.VK_2) {
                chosenX = 2;
            } else if (keyEvent.getKeyCode() == KeyEvent.VK_3) {
                chosenX = 3;
            } else if (keyEvent.getKeyCode() == KeyEvent.VK_4) {
                chosenX = 4;
            }

            if (keyEvent.getKeyCode() == KeyEvent.VK_Q) {
                chosenY = 0;
            } else if (keyEvent.getKeyCode() == KeyEvent.VK_W) {
                chosenY = 1;
            } else if (keyEvent.getKeyCode() == KeyEvent.VK_E) {
                chosenY = 2;
            } else if (keyEvent.getKeyCode() == KeyEvent.VK_R) {
                chosenY = 3;
            } else if (keyEvent.getKeyCode() == KeyEvent.VK_T) {
                chosenY = 4;
            }

            if(chosenX == -1) {
                chosenX = previousGridCoordinate.x;
            }

            if(chosenY == -1) {
                chosenY = previousGridCoordinate.y;
            }

            if(!(chosenX == previousGridCoordinate.x && chosenY == previousGridCoordinate.y)) {
                Pose2d startPose = bot.getPose();

                TrajectorySequenceBuilder builder = new TrajectorySequenceBuilder(startPose,
                        new MecanumVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.TRACK_WIDTH),
                        new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL),
                        DriveConstants.MAX_ANG_VEL, DriveConstants.MAX_ANG_ACCEL);

                Vector2i gridCoordinateDistance = new Vector2i(Math.abs(previousGridCoordinate.x - chosenX),
                        previousGridCoordinate.y - chosenY);
                boolean xFirst = gridCoordinateDistance.x >= gridCoordinateDistance.y;

                Vector2i chosen = new Vector2i(chosenX, chosenY);
                Vector2d finalPosition = gridToRRCoordinate(chosen);

                Vector2d snapped = gridToRRCoordinate(RRToGridCoordinate(startPose.vec()));
                if(xFirst) {
                    snapped = new Vector2d(startPose.getX(), snapped.getY());
                } else {
                    snapped = new Vector2d(snapped.getX(), startPose.getY());
                }

                if(!snapped.equals(startPose.vec())) {
                    builder.strafeTo(snapped);
                }

                if(xFirst) {
                    Vector2d first = gridToRRCoordinate(new Vector2i(chosen.x, previousGridCoordinate.y));
                    if(!first.equals(snapped)) {
                        builder.strafeTo(first);
                    }

                    if(!(gridCoordinateDistance.y == 0)) {
                        builder.strafeTo(finalPosition);
                    }
                } else {
                    Vector2d first = gridToRRCoordinate(new Vector2i(previousGridCoordinate.x, chosen.y));

                    if(!first.equals(snapped)) {
                        builder.strafeTo(first);
                    }

                    if(!(gridCoordinateDistance.x == 0)) {
                        builder.strafeTo(finalPosition);
                    }
                }

                builder.addDisplacementMarker(bot::pause);

                builder.strafeTo(new Vector2d(10100,10100));

                bot.setTrajectoryProgressSeconds(0);
                bot.followTrajectorySequence(builder.build());
                bot.unpause();

            }
        }

        @Override
        public void keyReleased(KeyEvent keyEvent) {}
    }

    public static Vector2d gridToRRCoordinate(Vector2i position) {
        return new Vector2d((position.x * 24) - 60, (position.y * 24) - 60);
    }

    public static Vector2i RRToGridCoordinate(Vector2d position) {
        return new Vector2i((int)(position.getX() + 60) / 24, (int)(position.getY() + 60) / 24);
    }
}

