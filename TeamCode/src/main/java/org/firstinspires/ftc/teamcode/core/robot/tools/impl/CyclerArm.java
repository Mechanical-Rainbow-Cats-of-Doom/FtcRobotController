package org.firstinspires.ftc.teamcode.core.robot.tools.impl;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.*;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.core.robot.util.EncoderNames;
import org.firstinspires.ftc.teamcode.roadrunner.util.Encoder;
import org.jetbrains.annotations.NotNull;

import java.util.ArrayDeque;
import java.util.Arrays;

@Config
public class CyclerArm {
    public static int bottomFinishedBound = 25, topFinishedBound = 9500;

    private final ArrayDeque<CRServo> servos;
    private final ArrayDeque<Encoder> encoders;
    private final Telemetry telemetry;

    private boolean extended = false;
    public CyclerArm(@NotNull HardwareMap hardwareMap, Telemetry telemetry) {
        final CRServo bottomServo = hardwareMap.get(CRServo.class, "bottom");
        bottomServo.setDirection(DcMotorSimple.Direction.REVERSE);
        final CRServo topServo = hardwareMap.get(CRServo.class, "top");
        topServo.setDirection(DcMotorSimple.Direction.FORWARD);
        this.servos = new ArrayDeque<>(Arrays.asList(
                topServo,
                bottomServo
        ));
        this.telemetry = telemetry;
        final Encoder topEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, EncoderNames.topArm));
        final Encoder bottomEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, EncoderNames.bottomArm));
        bottomEncoder.setDirection(Encoder.Direction.FORWARD);
        topEncoder.setDirection(Encoder.Direction.REVERSE);
        this.encoders = new ArrayDeque<>(Arrays.asList(
                topEncoder,
                new Encoder(hardwareMap.get(DcMotorEx.class, EncoderNames.bottomArm))
        ));
    }

    public void update() {
        servos.forEach(servo -> servo.setPower(extended ? 1 : -1));
        byte a = 0;
        for (CRServo servo : servos) {
            telemetry.addData("servo power " + ++a, servo.getPower());
        }
        telemetry.addData("Cycler extended", extended);
        byte i = 0; // byte to annoy fin
        for (Encoder encoder : encoders) {
            telemetry.addData("servo " + ++i, encoder.getCurrentPosition()); // prefix increment also to annoy fin
        }
    }

    public void setExtended(boolean extended) {
        this.extended = extended;
    }
    
    public boolean getExtended() {
        return extended;
    }

    public boolean isBusy() {
        for (Encoder encoder : encoders) {
            final int curVal = encoder.getCurrentPosition();
            if (curVal <= bottomFinishedBound || curVal >= topFinishedBound) return false;
        }
        return true;
    }
}
