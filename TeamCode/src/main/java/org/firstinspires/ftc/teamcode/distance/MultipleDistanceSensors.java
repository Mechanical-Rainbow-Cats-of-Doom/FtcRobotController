package org.firstinspires.ftc.teamcode.distance;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.Arrays;
import java.util.stream.IntStream;

public class MultipleDistanceSensors {
    private final int[][] distances;
    private final SEN0304DistanceSensor[] sensors;
    final Thread updateThread;

    public MultipleDistanceSensors(SEN0304DistanceSensor[] sensors, LinearOpMode opmode) {
        this.sensors = sensors;
        this.distances = new int[4][sensors.length];
        Arrays.fill(distances, new int[]{10, 10, 10, 10});

        this.updateThread = new Thread(() -> {
            while (!opmode.isStopRequested()) { // ask fin what to put here idk
                for (int val = 0; val < distances[0].length; val++) {
                    for (int i = 0; i < sensors.length; i++) {
                        sensors[i].readDistance();
                        try {
                            //noinspection BusyWait
                            Thread.sleep(20);
                        } catch (InterruptedException ignored) {}
                        distances[i][val] = sensors[(i > 0 ? i - 1 : sensors.length - 1)].getDistance();
                    }

                }
            }
        });

        updateThread.start();
    }

    public MultipleDistanceSensors(HardwareMap hardwareMap, LinearOpMode opmode) {
        this(new SEN0304DistanceSensor[]{
                hardwareMap.get(SEN0304DistanceSensor.class, "frontSensor"),
                hardwareMap.get(SEN0304DistanceSensor.class, "rightSensor"),
                hardwareMap.get(SEN0304DistanceSensor.class, "backSensor"),
                hardwareMap.get(SEN0304DistanceSensor.class, "leftSensor")
        }, opmode);
    }

    //takes 80*repetitions milliseconds
    public boolean isObject(int side, int minimumDistance){
        int distance = getDistance(side);
        return distance != -1 && distance < minimumDistance;
    }

    public int getDistance(int side) {
        return IntStream.of(distances[side]).filter((x) -> x==-1).count() != 0 ? -1 : IntStream.of(distances[side]).sum() / 4;
    }

    public void init() {
        updateThread.start();
    }
}
