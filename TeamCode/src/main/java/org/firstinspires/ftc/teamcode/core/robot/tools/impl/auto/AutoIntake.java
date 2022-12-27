package org.firstinspires.ftc.teamcode.core.robot.tools.impl.auto;

import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.core.thread.EventHelper;

public class AutoIntake {
    private final EventHelper eventHelper;
    private final CRServo servo;
    public AutoIntake(EventHelper eventHelper, CRServo servo) {
        this.eventHelper = eventHelper;
        this.servo = servo;
    }
}
