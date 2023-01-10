package org.firstinspires.ftc.teamcode.core.robot.util;

public class AutoStorage {
    // stored variables
    public static double delay = 0;
    public static Side side = Side.RED;


    // Side methods
    public enum Side {
        RED(true),
        BLUE(false);

        private boolean value;

        Side(boolean value) {
            this.value = value;
        }

        public boolean getSide() {
            return value;
        }

        public void changeSide(){
            this.value = !this.value;
        }

        public void setSide(boolean side){
            this.value = side;
        }
    }

    public static void changeSide() {
        AutoStorage.side.changeSide();
    }

    public static void setSide(boolean side){
        AutoStorage.side.setSide(side);
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


}
