package org.firstinspires.ftc.teamcode.core.robot.util;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.BangBang;
import com.ThermalEquilibrium.homeostasis.Parameters.BangBangParameters;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.roadrunner.util.Encoder;
import org.jetbrains.annotations.NotNull;


@Config
public class WeirdPIDServo {
    public static double floor = 0.07;
    public static int busyRange = 8;
    protected int targetPosition = 0;
    private final Encoder encoder;
    public final CRServo servo;
    protected final Telemetry telemetry;
    protected final BangBang controller;
    protected final BangBangParameters coefficients;
    protected final String servoName;
    public double power = 0;
    private int offset;
    public WeirdPIDServo(HardwareMap hardwareMap, Telemetry telemetry, String servoName, String encoderName,
                         BangBangParameters coefficients, Encoder.Direction encoderDirection,
                         DcMotorSimple.Direction servoDirection) {
        this.encoder = new Encoder(hardwareMap.get(DcMotorEx.class, encoderName));
        this.encoder.setDirection(encoderDirection);
        this.servo = hardwareMap.get(CRServo.class, servoName);
        this.servo.setDirection(servoDirection);
        this.servoName = servoName;
        this.coefficients = coefficients;
        this.controller = new BangBang(coefficients);
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
        power = controller.calculate(targetPosition, curPos);
        servo.setPower(power < 0 ? Math.min(-floor, power) : Math.max(floor, power));
        telemetry.addData(servoName + " power", power);
        telemetry.addData(servoName + " error", curPos - targetPosition);
        telemetry.addData(servoName + " target", targetPosition);
    }

    public void setTargetPosition(int targetPosition) {
        this.targetPosition = targetPosition;
    }
    public double getPower() {
        return power;
    }
    public int getTargetPosition() {
        return targetPosition;
    }
    
    public void updateCoefficients(@NotNull BangBangParameters bangBangParameters) {
        coefficients.maxOutput = bangBangParameters.maxOutput;
        coefficients.hysteresis = bangBangParameters.hysteresis;
    }

    public boolean isBusy() {
        return getPower() != 0;
    }
}
