package org.firstinspires.ftc.teamcode.core.robot.util;

import com.qualcomm.robotcore.util.ElapsedTime;

import androidx.annotation.NonNull;

public class AutoStorage {
    // stored variables
    private static double delay = 0;
    private static Side side = Side.RED;

    private static int cones = 3;

    // Side methods
    public enum Side {
        RED(true),
        BLUE(false);

        private final boolean value;

        Side(boolean value) {
            this.value = value;
        }

        public boolean getSide() {
            return value;
        }

    }
    public static void changeSide() {
        side = side == Side.RED ? Side.BLUE : Side.RED;
    }

    public static boolean getSide() {
        return (AutoStorage.side.getSide());
    }

    // Delay methods

    public static void setDelay(double seconds) {
        AutoStorage.delay = seconds;
    }
    public static void addDelay(double seconds) {
        AutoStorage.delay += seconds;
    }

    public static void subtractDelay(double seconds) {
        AutoStorage.delay = Math.max(AutoStorage.delay - seconds, 0);
    }

    public static double getDelay() {
        return delay;
    }

    public static void waitForDelay(@NonNull ElapsedTime timer) {
        while (timer.seconds() < delay) {
            // waiting...
        }
    }

    // Cone methods

    public static void incrementCones() {
        cones++;
    }

    public static void decrementCones() {
        if(cones > 0) {
            cones--;
        }
    }

    public static int getCones() {
        return cones;
    }
}
