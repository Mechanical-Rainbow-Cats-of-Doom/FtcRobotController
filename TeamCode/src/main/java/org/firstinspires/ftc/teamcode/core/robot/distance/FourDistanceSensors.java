package org.firstinspires.ftc.teamcode.core.robot.distance;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.util.ElapsedTime;

public class FourDistanceSensors {
    private int[] distances = {0, 0, 0 ,0};
    private final SEN0304DistanceSensor[] sensors;
    final Thread updateThread;
    public FourDistanceSensors(HardwareMap hardwareMap, Thread updateThread) {
        this.sensors = new SEN0304DistanceSensor[]{
                new SEN0304DistanceSensor(hardwareMap.get(I2cDeviceSynch.class, "frontSensor")),
                new SEN0304DistanceSensor(hardwareMap.get(I2cDeviceSynch.class, "rightSensor")),
                new SEN0304DistanceSensor(hardwareMap.get(I2cDeviceSynch.class, "leftSensor")),
                new SEN0304DistanceSensor(hardwareMap.get(I2cDeviceSynch.class, "backSensor"))
        };
        this.updateThread = new Thread(() -> {
            final ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
            timer.reset();
            while (!updateThread.isInterrupted()) { // ask fin what to put here idk
                for (int i = 0; i < sensors.length; i++) {
                    sensors[(i < 3 ? i + 1 : 0)].readDistance();
                    //need something to check if timer is greater than 40 ms before moving on
                    timer.reset();
                    sensors[i].getDistance();
                }
            }
        });
    }
    public void init() {
        updateThread.start();
    }
}
