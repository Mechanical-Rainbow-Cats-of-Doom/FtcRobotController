package com.mrcod.meepmeep;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySequence;
import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySequenceBuilder;

public class CoordinateUtil {
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

    public static TrajectorySequence trajectoryTo(Pose2d startPose, Vector2i chosenPosition, double heading) {
        Vector2i currentGridCoordinate = RRToGridCoordinate(startPose.vec());

        TrajectorySequenceBuilder builder = new TrajectorySequenceBuilder(startPose,
                new MecanumVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.TRACK_WIDTH),
                new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL),
                DriveConstants.MAX_ANG_VEL, DriveConstants.MAX_ANG_ACCEL);

        Vector2i gridCoordinateDistance = new Vector2i(Math.abs(currentGridCoordinate.x - chosenPosition.x),
                currentGridCoordinate.y - chosenPosition.y);
        boolean xFirst = gridCoordinateDistance.x >= gridCoordinateDistance.y;

        Vector2d snapped = gridToRRCoordinate(RRToGridCoordinate(startPose.vec()));
        if(xFirst) {
            snapped = new Vector2d(startPose.getX(), snapped.getY());
        } else {
            snapped = new Vector2d(snapped.getX(), startPose.getY());
        }

        Vector2d first = gridToRRCoordinate(xFirst ? new Vector2i(chosenPosition.x, currentGridCoordinate.y) : new Vector2i(currentGridCoordinate.x, chosenPosition.y));
        Vector2d finalPosition = gridToRRCoordinate(chosenPosition);

        if(!snapped.equals(startPose.vec())) {
            builder.strafeTo(snapped);
        }

        if(!first.equals(snapped)) {
            if(snapped.distTo(first) > first.distTo(finalPosition)) {
                builder.lineToLinearHeading(new Pose2d(first, heading));
            } else {
                builder.strafeTo(first);
            }
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

        return builder.build();
    }

    public static Vector2d gridToRRCoordinate(Vector2i position) {
        return new Vector2d((position.x * 24) - 60, (position.y * 24) - 60);
    }

    public static Vector2i RRToGridCoordinate(Vector2d position) {
        return new Vector2i((int)Math.round((position.getX() + 60) / 24), (int)Math.round((position.getY() + 60) / 24));
    }
}
