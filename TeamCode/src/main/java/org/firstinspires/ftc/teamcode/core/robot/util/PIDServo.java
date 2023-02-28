package org.firstinspires.ftc.teamcode.core.robot.util;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.PIDEx;
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficientsEx;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.roadrunner.util.Encoder;
import org.jetbrains.annotations.NotNull;

import java.lang.reflect.Method;

@Config
public class PIDServo {
    public static int busyRange = 8;
    public int targetPosition = 0;
    public final Encoder encoder;
    public final CRServo servo;
    public final Telemetry telemetry;
    public final PIDEx controller;
    public final PIDCoefficientsEx coefficients;
    private final String servoName;
    private int offset;
    public PIDServo(HardwareMap hardwareMap, Telemetry telemetry, String servoName, String encoderName,
                    PIDCoefficientsEx coefficients, Encoder.Direction encoderDirection,
                    DcMotorSimple.Direction servoDirection) {
        this.encoder = new Encoder(hardwareMap.get(DcMotorEx.class, encoderName));
        this.encoder.setDirection(encoderDirection);
        this.servo = hardwareMap.get(CRServo.class, servoName);
        this.servo.setDirection(servoDirection);
        this.servoName = servoName;
        this.coefficients = coefficients;
        this.controller = new PIDEx(coefficients);
        this.telemetry = telemetry;
        zero();
    }

    public void zero() {
        offset = encoder.getCurrentPosition();
    }

    public int getCurrentPosition() {
        return encoder.getCurrentPosition() - offset;
    }

    public void update() {
        final int curPos = getCurrentPosition();
        final double power = controller.calculate(targetPosition, curPos);
        servo.setPower(power);
        telemetry.addData(servoName + " power", power);
        telemetry.addData(servoName + " error", curPos);
        telemetry.addData(servoName + " target", targetPosition);
    }

    public void setTargetPosition(int targetPosition) {
        this.targetPosition = targetPosition;
    }

    public int getTargetPosition() {
        return targetPosition;
    }
    
    public void updateCoefficients(@NotNull PIDCoefficientsEx newCoefficients) {
        coefficients.Kp = newCoefficients.Kp;
        coefficients.Ki = newCoefficients.Ki;
        coefficients.Kd = newCoefficients.Kd;
        coefficients.lowPassGain = newCoefficients.lowPassGain;
        coefficients.stabilityThreshold = newCoefficients.stabilityThreshold;
        coefficients.maximumIntegralSum = newCoefficients.maximumIntegralSum;
    }

    public boolean isBusy() {
        final int curPos = getCurrentPosition();
        return targetPosition - busyRange <= curPos && curPos <= targetPosition + busyRange;
    }
}
