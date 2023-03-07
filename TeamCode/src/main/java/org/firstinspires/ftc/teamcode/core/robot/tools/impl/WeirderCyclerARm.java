package org.firstinspires.ftc.teamcode.core.robot.tools.impl;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.core.robot.util.EncoderNames;
import org.firstinspires.ftc.teamcode.core.robot.util.PIDServo;
import org.firstinspires.ftc.teamcode.roadrunner.util.Encoder;

@Config
public class WeirderCyclerARm {
    /*
    public static class WeirdPIDServo extends PIDServo {
        public WeirdPIDServo(HardwareMap hardwareMap, Telemetry telemetry, String servoName, String encoderName, BangBangParameters coefficients, Encoder.Direction encoderDirection, DcMotorSimple.Direction servoDirection) {
            super(hardwareMap, telemetry, servoName, encoderName, coefficients, encoderDirection, servoDirection);
        }

        @Override
        public void update() {
            final int curPos = getCurrentPosition();
            power = controller.calculate(targetPosition + (coefficients.hysteresis / 2), curPos);
            servo.setPower(power < 0 ? Math.min(-bottomFloor, power) : Math.max(bottomFloor, power));
            telemetry.addData(servoName + " power", power);
            telemetry.addData(servoName + " error", curPos - targetPosition);
            telemetry.addData(servoName + " target", targetPosition);
        }
    }
    private final PIDServo top, bottom;
    public static double floor = 0.07, bottomFloor = 0.05;
    public static double topHysteresis = 500, bottomHysteresis = 150;
    public static double topMaxOutput = 1, bottomMaxOutput = 0.5;
    public WeirderCyclerARm(HardwareMap hardwareMap, Telemetry telemetry) {
        this.top = new PIDServo(hardwareMap, telemetry, "top", EncoderNames.topArm,
                new BangBangParameters(topMaxOutput, topHysteresis), Encoder.Direction.REVERSE, DcMotorSimple.Direction.FORWARD);
        this.bottom = new WeirdPIDServo(hardwareMap, telemetry, "bottom", EncoderNames.bottomArm,
                new BangBangParameters(bottomMaxOutput, bottomHysteresis), Encoder.Direction.FORWARD, DcMotorSimple.Direction.REVERSE);
    }

    public void debugUpdate() {
        top.updateCoefficients(new BangBangParameters(topMaxOutput, topHysteresis));
        bottom.updateCoefficients(new BangBangParameters(bottomMaxOutput, bottomHysteresis));
        update();
    }

    public void update() {
        top.update();
        double power = top.getPower();
        if (power != 0) {
            bottom.servo.setPower(power);
        } else {
            bottom.update();
        }
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

    }

    public void setTopTargetPosition(int target) {
        top.setTargetPosition(target);
    }

    public void setBottomTargetPosition(int target) {
        bottom.setTargetPosition(target);
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
        return bottom.getCurrentPosition();
    }

    public int getBottomTargetPosition() {
        return bottom.getTargetPosition();
    }

    public boolean isTopBusy() {
        return top.isBusy();
    }

    public boolean isBottomBusy() {
        return bottom.isBusy();
    }

    public boolean isBusy() {
        return isTopBusy() || isBottomBusy();
    }

    public void zero() {
        top.zero();
        bottom.zero();
    }
    */
}
