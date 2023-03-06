package org.firstinspires.ftc.teamcode.core.robot.tools.impl;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.BangBang;
import com.ThermalEquilibrium.homeostasis.Parameters.BangBangParameters;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.core.robot.util.EncoderNames;
import org.firstinspires.ftc.teamcode.core.robot.util.WeirdPIDServo;
import org.firstinspires.ftc.teamcode.roadrunner.util.Encoder;

@Config
public class WeirdCyclerArm {
    private final WeirdPIDServo top;
    private final Encoder bottomEncoder;
    private final CRServo bottomServo;
    private int targetPosition = 0;
    public static double floor = 0.05, bottomFloor = 0.15;
    public static double hysteresis = 50;
    public static double maxOutput = 1;
    public static double servoFollowPercent = 0, servoCatchUpSpeed = 0, holdPower = 0.008;
    private boolean needsToMoveUp = true, hasntBeenSet = true, bottomServoHolding = true;
    private int offset;
    public WeirdCyclerArm(HardwareMap hardwareMap, Telemetry telemetry) {
        this.top = new WeirdPIDServo(hardwareMap, telemetry, "top", EncoderNames.topArm,
                new BangBangParameters(maxOutput, hysteresis), Encoder.Direction.REVERSE, DcMotorSimple.Direction.FORWARD);
        this.bottomEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, EncoderNames.bottomArm));
        this.bottomEncoder.setDirection(Encoder.Direction.FORWARD);
        this.bottomServo = hardwareMap.get(CRServo.class, "bottom");
        this.bottomServo.setDirection(DcMotorSimple.Direction.REVERSE);
        offset = bottomEncoder.getCurrentPosition();
    }

    public void debugUpdate() {
        top.updateCoefficients(new BangBangParameters(maxOutput, hysteresis));
        update();
    }

    public void update() {
        top.update();
        double power = top.getPower();
        bottomServo.setPower(power < 0 ? Math.min(-bottomFloor, power) : Math.max(bottomFloor, power));
        /*
        if (power != 0) {
            bottomServo.setPower(power*(servoFollowPercent/100));
            bottomServoHolding = false;
            hasntBeenSet = true;
        } else {
            if (hasntBeenSet) {
                needsToMoveUp = bottomEncoder.getCurrentPosition() < getBottomTargetPosition();
                hasntBeenSet = false;
            }
            if (needsToMoveUp ? bottomEncoder.getCurrentPosition() < getBottomTargetPosition() : bottomEncoder.getCurrentPosition() > getBottomTargetPosition()) {
                bottomServo.setPower(needsToMoveUp ? servoCatchUpSpeed : -servoCatchUpSpeed);
            } else {
                bottomServoHolding = true;
                bottomServo.setPower(holdPower);
            }

        }
         */
    }

    public void setTopTargetPosition(int target) {
        top.setTargetPosition(target);
    }

    public void setBottomTargetPosition(int target) {
        targetPosition = target;
    }

    public void setTargetPosition(int target) {
        setTopTargetPosition(target);
        setBottomTargetPosition(target);
    }

    public int getTopCurrentPosition() {
        return top.getCurrentPosition();
    }

    public int getTopTargetPosition() {
        return top.getTargetPosition();
    }

    public int getBottomCurrentPosition() {
        return bottomEncoder.getCurrentPosition();
    }

    public int getBottomTargetPosition() {
        return targetPosition;
    }

    public boolean isTopBusy() {
        return top.isBusy();
    }

    public boolean isBottomBusy() {
        return !bottomServoHolding;
    }

    public boolean isBusy() {
        return isTopBusy() || isBottomBusy();
    }

    public void zero() {
        top.zero();
        offset = bottomEncoder.getCurrentPosition();
    }
}
