package org.firstinspires.ftc.teamcode.roadrunner.util;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

public class MirroringUtil {
    /**
     * Mirrors a pose across the X axis.
     */
    public static Pose2d mirrorX(Pose2d input) {
        return new Pose2d(-input.getX(), input.getY(),
                Math.PI-input.getHeading());
    }

    /**
     * Mirrors a vector across the X axis.
     */
    public static Vector2d mirrorX(Vector2d input) {
        return new Vector2d(-input.getX(), input.getY());
    }

    /**
     * Conditionally mirrors a pose across the X axis.
     */
    public static Pose2d cMirrorX(Pose2d input, boolean condition) {
        return condition ? mirrorX(input) : input;
    }

    /**
     * Conditionally mirrors a vector across the X axis.
     */
    public static Vector2d cMirrorX(Vector2d input, boolean condition) {
        return condition ? mirrorX(input) : input;
    }
}
