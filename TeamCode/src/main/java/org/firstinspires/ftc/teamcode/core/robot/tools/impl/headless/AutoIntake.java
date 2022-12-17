package org.firstinspires.ftc.teamcode.core.robot.tools.impl.headless;

import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.core.robot.tools.api.headless.HeadlessToggleableTool;
import org.firstinspires.ftc.teamcode.core.thread.EventHelper;
import org.firstinspires.ftc.teamcode.core.thread.event.impl.RunAtTimeEvent;

public class AutoIntake extends HeadlessToggleableTool<CRServo> {
    private final EventHelper eventHelper;
    private RunAtTimeEvent event = null;
    public AutoIntake(EventHelper eventHelper, CRServo motor, double power) {
        super(motor, power);
        this.eventHelper = eventHelper;
    }

    @Override
    public void on() {
        if(event != null) {
            event.cancel();
        }
        super.on();
    }

    @Override
    public void off() {
        motor.setPower(-0.1);
        event = new RunAtTimeEvent(() -> {
            motor.setPower(0);
        }, System.currentTimeMillis() + 1000);
        eventHelper.addEvent(event);
    }
}
