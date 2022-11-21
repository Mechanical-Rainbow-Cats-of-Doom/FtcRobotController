package org.firstinspires.ftc.teamcode.core.robot.distance;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;
import java.util.stream.IntStream;

public class FourDistanceSensors {
    private final int[][] distances = { // sensor, last 4 values
            {100, 100, 100, 100},
            {100, 100, 100, 100},
            {100, 100, 100, 100},
            {100, 100, 100, 100}
    };
    private final SEN0304DistanceSensor[] sensors;
    final Thread updateThread;

    public FourDistanceSensors(HardwareMap hardwareMap) {
        this.sensors = new SEN0304DistanceSensor[]{
                hardwareMap.get(SEN0304DistanceSensor.class, "frontSensor"),
                hardwareMap.get(SEN0304DistanceSensor.class, "rightSensor"),
                hardwareMap.get(SEN0304DistanceSensor.class, "backSensor"),
                hardwareMap.get(SEN0304DistanceSensor.class, "leftSensor")
        };

        this.updateThread = new Thread(() -> {
            while (true) { // ask fin what to put here idk
                for (int val = 0; val < distances[0].length; val++) {
                    for (int i = 0; i < sensors.length; i++) {
                        sensors[i].readDistance();
                        try {
                            //noinspection BusyWait
                            Thread.sleep(20);
                        } catch (InterruptedException ignored) {}
                        distances[i][val] = sensors[(i > 0 ? i - 1 : 3)].getDistance();
                    }

                }
            }
        });
    }

    //takes 80*repetitions milliseconds
    public boolean isObject(int side, int minimumDistance){
        int distance = getDistance(side);
        return distance != -1 && distance < minimumDistance;
    }

    public int getDistance(int side) {
        // TODO fix this garbage and make sure it doesn't count -1s
        IntStream stream = IntStream.of(distances[side]).filter((i) -> i != -1);
        List<Integer> distances = stream.boxed().collect(Collectors.toList());
        int count = (int)stream.count();
        return count == 0 ? -1 : stream.sum() / (int)stream.count();
    }

    public void init() {
        updateThread.start();
    }
}
