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
import java.util.Vector;

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
        roadRunnerBot.setLooping(false);

        return roadRunnerBot;
    }

    static class Vector2i {
        public final int x;
        public final int y;

        public Vector2i(int x, int y) {
            this.x = x;
            this.y = y;
        }

        public double distance(Vector2i other) {
            return Math.sqrt(Math.pow(other.x - this.x, 2) + Math.pow(other.y - this.y, 2));
        }
    }


    static class BotKeyListener implements KeyListener {
        public final RoadRunnerBotEntity bot;
        public Vector2i previousGridCoordinate;
        public Vector2i choice;
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

            if (keyEvent.getKeyCode() == KeyEvent.VK_1) {
                chosenX = 0;
            } else if (keyEvent.getKeyCode() == KeyEvent.VK_2) {
                chosenX = 1;
            } else if (keyEvent.getKeyCode() == KeyEvent.VK_3) {
                chosenX = 2;
            } else if (keyEvent.getKeyCode() == KeyEvent.VK_4) {
                chosenX = 3;
            } else if (keyEvent.getKeyCode() == KeyEvent.VK_5) {
                chosenX = 4;
            }  else if (keyEvent.getKeyCode() == KeyEvent.VK_6) {
                chosenX = 5;
            } else if (keyEvent.getKeyCode() == KeyEvent.VK_Q) {
                chosenY = 0;
            } else if (keyEvent.getKeyCode() == KeyEvent.VK_W) {
                chosenY = 1;
            } else if (keyEvent.getKeyCode() == KeyEvent.VK_E) {
                chosenY = 2;
            } else if (keyEvent.getKeyCode() == KeyEvent.VK_R) {
                chosenY = 3;
            } else if (keyEvent.getKeyCode() == KeyEvent.VK_T) {
                chosenY = 4;
            } else if (keyEvent.getKeyCode() == KeyEvent.VK_Y) {
                chosenY = 5;
            } else if (keyEvent.getKeyCode() == KeyEvent.VK_A) {
                heading -= Math.PI / 2;
                if(heading <= 0) {
                    heading += Math.PI;
                }
            } else if (keyEvent.getKeyCode() == KeyEvent.VK_D) {
                heading += Math.PI / 2;
                if(heading >= Math.PI * 2) {
                    heading -= Math.PI;
                }
            }

            if(chosenX == -1) {
                chosenX = choice.x;
            }

            if(chosenY == -1) {
                chosenY = choice.y;
            }

            choice = new Vector2i(chosenX, chosenY);

            Vector2i currentGridCoordinate = RRToGridCoordinate(bot.getPose().vec());
            if(!(chosenX == currentGridCoordinate.x && chosenY == currentGridCoordinate.y) && keyEvent.getKeyCode() == KeyEvent.VK_ENTER) {
                goTo(currentGridCoordinate);
            }
        }

        @Override
        public void keyReleased(KeyEvent keyEvent) {}

        public void goTo(Vector2i currentGridCoordinate) {
            long startTime = System.currentTimeMillis();
            Pose2d startPose = bot.getPose();

//                boolean positiveTurn = false;
//                if(startPose.getHeading() != heading) {
//                    double direction1 = Math.abs(startPose.getHeading() - heading);
//                    double direction2 = Math.abs(heading - startPose.getHeading());
//                    positiveTurn = direction1 > direction2;
//                }

            TrajectorySequenceBuilder builder = new TrajectorySequenceBuilder(startPose,
                    new MecanumVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.TRACK_WIDTH),
                    new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL),
                    DriveConstants.MAX_ANG_VEL, DriveConstants.MAX_ANG_ACCEL);

            Vector2i gridCoordinateDistance = new Vector2i(Math.abs(currentGridCoordinate.x - choice.x),
                    currentGridCoordinate.y - choice.y);
            boolean xFirst = gridCoordinateDistance.x >= gridCoordinateDistance.y;

            Vector2d snapped = gridToRRCoordinate(RRToGridCoordinate(startPose.vec()));
            if(xFirst) {
                snapped = new Vector2d(startPose.getX(), snapped.getY());
            } else {
                snapped = new Vector2d(snapped.getX(), startPose.getY());
            }

            Vector2d first = gridToRRCoordinate(xFirst ? new Vector2i(choice.x, currentGridCoordinate.y) : new Vector2i(currentGridCoordinate.x, choice.y));
            Vector2d finalPosition = gridToRRCoordinate(choice);

            long startWeighting = System.nanoTime();

            double normalizedWeightSnapped;
            double normalizedWeightFirst;

            {
                double weightSnapped = startPose.vec().distTo(snapped);
                double weightFirst = snapped.distTo(first);
                double weightFinal = first.distTo(finalPosition);

                double normalizingFactor = (weightSnapped + weightFirst + weightFinal);
                normalizedWeightSnapped = weightSnapped / normalizingFactor;
                normalizedWeightFirst = weightFirst / normalizingFactor;
            }

            double snappedHeading = ((heading - startPose.getHeading()) * normalizedWeightSnapped) + startPose.getHeading();
            double firstHeading = ((heading - startPose.getHeading()) * (normalizedWeightSnapped + normalizedWeightFirst)) + startPose.getHeading();

            System.out.println("Weighting and Heading calculations took " + (System.nanoTime() - startWeighting) + " ns");

            long startBuilding = System.nanoTime();

            if(!snapped.equals(startPose.vec())) {
                builder.lineToLinearHeading(new Pose2d(snapped, snappedHeading));
            }

            if(!first.equals(snapped)) {
                builder.lineToLinearHeading(new Pose2d(first, firstHeading));
            }

            if(xFirst) {
                if(!(gridCoordinateDistance.y == 0)) {
                    builder.lineToLinearHeading(new Pose2d(finalPosition, heading));
                }
            } else {
                if(!(gridCoordinateDistance.x == 0)) {
                    builder.lineToLinearHeading(new Pose2d(finalPosition, heading));
                }
            }

            System.out.println("Building took " + (System.nanoTime() - startBuilding) + " ns");

            bot.followTrajectorySequence(builder.build());
            bot.setTrajectoryProgressSeconds(0);
            bot.setLooping(false);

            previousGridCoordinate = choice;
            System.out.println("Creating trajectory took " + (System.currentTimeMillis() - startTime) + " milliseconds.");
        }
    }

    public static Vector2d gridToRRCoordinate(Vector2i position) {
        return new Vector2d((position.x * 24) - 60, (position.y * 24) - 60);
    }

    public static Vector2i RRToGridCoordinate(Vector2d position) {
        return new Vector2i((int)(position.getX() + 60) / 24, (int)(position.getY() + 60) / 24);
    }
}

