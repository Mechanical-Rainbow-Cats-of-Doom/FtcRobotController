package org.firstinspires.ftc.teamcode.roadrunner.util;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.jetbrains.annotations.Contract;

public class MirroringUtil {
    /**
     * Mirrors a pose across the X axis.
     */
    @NonNull
    @Contract("_ -> new")
    public static Pose2d mirrorX(Pose2d input) {
        return new Pose2d(-input.getX(), input.getY(),
                Math.PI-input.getHeading());
    }

    /**
     * Mirrors a vector across the X axis.
     */
    @NonNull
    @Contract("_ -> new")
    public static Vector2d mirrorX(Vector2d input) {
        return new Vector2d(-input.getX(), input.getY());
    }

    /**
     * Conditionally mirrors a pose across the X axis.
     */
    @NonNull
    public static Pose2d cMirrorX(Pose2d input, boolean condition) {
        return condition ? mirrorX(input) : input;
    }

    /**
     * Conditionally mirrors a vector across the X axis.
     */
    @NonNull
    public static Vector2d cMirrorX(Vector2d input, boolean condition) {
        return condition ? mirrorX(input) : input;
    }

    /**
     * Mirrors across a pose the Y axis.
     */
    @NonNull
    @Contract("_ -> new")
    public static Pose2d mirrorY(Pose2d input) {
        return new Pose2d(input.getX(), -input.getY(),
                (2*Math.PI)-input.getHeading());
    }

    /**
     * Mirrors a vector across the Y axis.
     */
    @NonNull
    @Contract("_ -> new")
    public static Vector2d mirrorY(Vector2d input) {
        return new Vector2d(input.getX(), -input.getY());
    }

    /**
     * Conditionally mirrors a pose across the Y axis.
     */
    @NonNull
    public static Pose2d cMirrorY(Pose2d input, boolean condition) {
        return condition ? mirrorY(input) : input;
    }

    /**
     * Conditionally mirrors a vector across the Y axis.
     */
    @NonNull
    public static Vector2d cMirrorY(Vector2d input, boolean condition) {
        return condition ? mirrorY(input) : input;
    }
}
