package org.firstinspires.ftc.teamcode.core.robot.tools.impl;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.*;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.core.robot.util.EncoderNames;
import org.firstinspires.ftc.teamcode.core.robot.util.ZeroMotorEncoder;
import org.firstinspires.ftc.teamcode.roadrunner.util.Encoder;
import org.jetbrains.annotations.NotNull;

import java.util.ArrayDeque;
import java.util.Arrays;

@Config
public class CyclerArm {
    public static int bottomFinishedBound = 25, topFinishedBound = 9500;
    private int topOffset, bottomOffset;
    private final ArrayDeque<CRServo> servos;
    private final Encoder top, bottom;
    private final Telemetry telemetry;

    private boolean extended = false;
    public CyclerArm(@NotNull HardwareMap hardwareMap, Telemetry telemetry, LinearOpMode opMode) {
        final CRServo bottomServo = hardwareMap.get(CRServo.class, "bottom");
        bottomServo.setDirection(DcMotorSimple.Direction.REVERSE);
        final CRServo topServo = hardwareMap.get(CRServo.class, "top");
        topServo.setDirection(DcMotorSimple.Direction.FORWARD);
        this.servos = new ArrayDeque<>(Arrays.asList(
                topServo,
                bottomServo
        ));
        this.telemetry = telemetry;
        top = new Encoder(hardwareMap.get(DcMotorEx.class, EncoderNames.topArm));
        bottom = new Encoder(hardwareMap.get(DcMotorEx.class, EncoderNames.bottomArm));
        bottom.setDirection(Encoder.Direction.FORWARD);
        bottomOffset = bottom.getCurrentPosition();
        top.setDirection(Encoder.Direction.REVERSE);
        topOffset = top.getCurrentPosition();
        Thread thread = new Thread(() -> {
            opMode.waitForStart();
            try {
                Thread.sleep(500);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            bottomOffset = bottom.getCurrentPosition();
            topOffset = top.getCurrentPosition();
        });
        thread.start();
    }

    public void update() {
        servos.forEach(servo -> servo.setPower(extended ? 1 : -1));
        byte a = 0;
        for (CRServo servo : servos) {
            telemetry.addData("servo power " + ++a, servo.getPower());
        }
        telemetry.addData("Cycler extended", extended);
        telemetry.addData("top", getTopPosition());
        telemetry.addData("bottom", getBottomPosition());
    }

    public void setExtended(boolean extended) {
        this.extended = extended;
    }
    
    public boolean getExtended() {
        return extended;
    }
    public int getTopPosition() {
        return top.getCurrentPosition() - topOffset;
    }
    public int getBottomPosition() {
        return bottom.getCurrentPosition() - bottomOffset;
    }
    public boolean isBusy() {
        final int topCurVal = getTopPosition();
        final int bottomCurVal = getBottomPosition();
        return !(topCurVal > bottomFinishedBound && topCurVal < topFinishedBound && (bottomCurVal > bottomFinishedBound && bottomCurVal < topFinishedBound));
    }
}
