package org.firstinspires.ftc.teamcode.core.robot.tools.impl;

import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficientsEx;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.firstinspires.ftc.teamcode.core.robot.util.EncoderNames;
import org.firstinspires.ftc.teamcode.core.robot.util.PIDServo;
import org.firstinspires.ftc.teamcode.roadrunner.util.Encoder;
import org.jetbrains.annotations.Contract;
import org.jetbrains.annotations.NotNull;

public class CyclerArm {
    private final PIDServo top, bottom;

    @Config
    public static class TopArm {
        public static double P = 0.6, I = 1.2, D = 0.075, stabilityThresh = 25, lowPassGain = 0.65, integralSumMax0IfNotUsed = 0;
        public static double getIntegralSumMax() {
            return integralSumMax0IfNotUsed == 0 ? 1/I : integralSumMax0IfNotUsed;
        }

        @Contract(" -> new")
        public static @NotNull PIDCoefficientsEx getCoefficients() {
            return new PIDCoefficientsEx(P, I, D, getIntegralSumMax(), stabilityThresh, lowPassGain);
        }
    }
    @Config
    public static class BottomArm {
        public static double P = 0.6, I = 1.2, D = 0.075, stabilityThresh = 25, lowPassGain = 0.65, integralSumMax0IfNotUsed = 0;
        public static double getIntegralSumMax() {
            return integralSumMax0IfNotUsed == 0 ? 1/I : integralSumMax0IfNotUsed;
        }

        @Contract(" -> new")
        public static @NotNull PIDCoefficientsEx getCoefficients() {
            return new PIDCoefficientsEx(P, I, D, getIntegralSumMax(), stabilityThresh, lowPassGain);
        }
    }

    public CyclerArm(HardwareMap hardwareMap, Telemetry telemetry) {
        this.top = new PIDServo(hardwareMap, telemetry, "top", EncoderNames.topArm,
                TopArm.getCoefficients(), Encoder.Direction.FORWARD, DcMotorSimple.Direction.FORWARD);
        this.bottom = new PIDServo(hardwareMap, telemetry, "bottom", EncoderNames.bottomArm,
                BottomArm.getCoefficients(), Encoder.Direction.FORWARD, DcMotorSimple.Direction.FORWARD);
    }

    public void debugUpdate() {
        top.updateCoefficients(TopArm.getCoefficients());
        bottom.updateCoefficients(BottomArm.getCoefficients());
        update();
    }

    public void update() {
        top.update();
        bottom.update();
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
