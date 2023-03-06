package org.firstinspires.ftc.teamcode.core.robot.tools.impl;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.core.robot.util.EncoderNames;
import org.firstinspires.ftc.teamcode.core.robot.util.PIDServo;
import org.firstinspires.ftc.teamcode.roadrunner.util.Encoder;
import org.jetbrains.annotations.Contract;
import org.jetbrains.annotations.NotNull;

@Config
public class CyclerArm {
    private final PIDServo top, bottom;
    public static int listSize = 10;
    public static boolean usingTopPID = false;
    @Config
    public static class TopArm {
        public static double P = 0.0005, I = 8e-13, D = 0.001, stabilityThresh = 25, lowPassGain = 0.65, integralSumMax0IfNotUsed = Integer.MAX_VALUE, Kpo = -0.0002, Kio = -4e-14;
        public static double getIntegralSumMax() {
            return integralSumMax0IfNotUsed == 0 ? 1/I : integralSumMax0IfNotUsed;
        }

        @Contract(" -> new")
        public static @NotNull PIDServo.PIDExEx.PIDCoefficientsExEx getCoefficients() {
            return new PIDServo.PIDExEx.PIDCoefficientsExEx(P, I, D, Kpo, Kio, getIntegralSumMax(), stabilityThresh, lowPassGain);
        }
    }
    @Config
    public static class BottomArm {
        public static double P = 0.0005, I = 0.0000000000008, D = 0.0001, stabilityThresh = 25, lowPassGain = 0.65, integralSumMax0IfNotUsed = Integer.MAX_VALUE, Kpo = -0.0002, Kio = -1e-13;
        public static double getIntegralSumMax() {
            return integralSumMax0IfNotUsed == 0 ? 1/I : integralSumMax0IfNotUsed;
        }

        @Contract(" -> new")
        public static @NotNull PIDServo.PIDExEx.PIDCoefficientsExEx getCoefficients() {
            return new PIDServo.PIDExEx.PIDCoefficientsExEx(P, I, D, Kpo, Kio, getIntegralSumMax(), stabilityThresh, lowPassGain);
        }
    }

    public CyclerArm(HardwareMap hardwareMap, Telemetry telemetry) {
        this.top = new PIDServo(hardwareMap, telemetry, "top", EncoderNames.topArm,
                TopArm.getCoefficients(), Encoder.Direction.REVERSE, DcMotorSimple.Direction.FORWARD);
        this.bottom = new PIDServo(hardwareMap, telemetry, "bottom", EncoderNames.bottomArm,
                BottomArm.getCoefficients(), Encoder.Direction.FORWARD, DcMotorSimple.Direction.REVERSE);
    }

    public void debugUpdate() {
        top.updateCoefficients(TopArm.getCoefficients());
        bottom.updateCoefficients(BottomArm.getCoefficients());
        update();
    }

    public void update() {
        top.update(getBottomTargetPosition(), getBottomCurrentPosition());
        bottom.update(getTopTargetPosition(), getTopCurrentPosition());
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
}
