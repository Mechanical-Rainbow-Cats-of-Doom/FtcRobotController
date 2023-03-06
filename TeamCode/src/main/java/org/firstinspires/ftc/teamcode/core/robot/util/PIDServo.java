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

@Config
public class PIDServo {
    public static class PIDExEx extends PIDEx {
        public PIDCoefficientsExEx coeffs;
        private double otherIntegralSum = 0, otherPreviousError = 0;
        public static class PIDCoefficientsExEx extends PIDCoefficientsEx {
            public double Kpo, Kio;
            public PIDCoefficientsExEx(double Kp, double Ki, double Kd, double Kpo, double Kio, double maximumIntegralSum, double stabilityThreshold, double lowPassGain) {
                super(Kp, Ki, Kd, maximumIntegralSum, stabilityThreshold, lowPassGain);
                this.Kpo = Kpo;
                this.Kio = Kio;
            }
        }
        public PIDExEx(PIDCoefficientsExEx coefficients) {
            super(coefficients);
            this.coeffs = coefficients;
        }
        private void integrateOther(double otherError, double dt) {
            if (crossOverDetected(otherError,otherPreviousError)) otherIntegralSum = 0;
            if (Math.abs(derivative) > basedCoefficients.stabilityThreshold) return;
            otherIntegralSum += ((otherError + otherPreviousError) / 2) * dt;
            if (Math.abs(otherIntegralSum) > basedCoefficients.maximumIntegralSum) {
                otherIntegralSum = Math.signum(otherIntegralSum) * basedCoefficients.maximumIntegralSum;
            }
        }
        public double calculate(double reference, double state, double otherReference, double otherState, Telemetry telemetry) {
            double dt = getDT();
            double error = calculateError(reference, state);
            double otherError = calculateError(otherReference, otherState);
            double derivative = calculateDerivative(error,dt);
            integrate(error,dt);
            integrateOther(otherError, dt);
            previousError = error;
            otherPreviousError = otherError;
            return (error * coeffs.Kp)
                    + (integralSum * coeffs.Ki)
                    + (derivative * coeffs.Kd)
                    + (otherError * coeffs.Kpo)
                    + (otherIntegralSum * coeffs.Kio);
        }
    }

    public static int busyRange = 8;
    public int targetPosition = 0;
    public final Encoder encoder;
    public final CRServo servo;
    public final Telemetry telemetry;
    public final PIDExEx controller;
    public final PIDExEx.PIDCoefficientsExEx coefficients;
    public final String servoName;
    public double power;
    public int offset;
    public PIDServo(HardwareMap hardwareMap, Telemetry telemetry, String servoName, String encoderName,
                    PIDExEx.PIDCoefficientsExEx coefficients, Encoder.Direction encoderDirection,
                    DcMotorSimple.Direction servoDirection) {
        this.encoder = new Encoder(hardwareMap.get(DcMotorEx.class, encoderName));
        this.encoder.setDirection(encoderDirection);
        this.servo = hardwareMap.get(CRServo.class, servoName);
        this.servo.setDirection(servoDirection);
        this.servoName = servoName;
        this.coefficients = coefficients;
        this.controller = new PIDExEx(coefficients);
        this.telemetry = telemetry;
        zero();
    }

    public void zero() {
        offset = encoder.getCurrentPosition();
    }

    public int getCurrentPosition() {
        return encoder.getCurrentPosition() - offset;
    }

    public void update(double otherTarget, double otherCurState) {
        final int curPos = getCurrentPosition();
        final double power = controller.calculate(targetPosition, curPos, otherTarget, otherCurState, telemetry);
        servo.setPower(power);
        telemetry.addData(servoName + " power", power);
        telemetry.addData(servoName + " error", curPos - targetPosition);
        telemetry.addData(servoName + " target", targetPosition);
        telemetry.addData(servoName+ " current", curPos);
    }

    public void setTargetPosition(int targetPosition) {
        this.targetPosition = targetPosition;
    }

    public int getTargetPosition() {
        return targetPosition;
    }
    
    public void updateCoefficients(@NotNull PIDExEx.PIDCoefficientsExEx newCoefficients) {
        controller.coeffs.Kp = newCoefficients.Kp;
        controller.coeffs.Ki = newCoefficients.Ki;
        controller.coeffs.Kd = newCoefficients.Kd;
        controller.coeffs.lowPassGain = newCoefficients.lowPassGain;
        controller.coeffs.stabilityThreshold = newCoefficients.stabilityThreshold;
        controller.coeffs.maximumIntegralSum = newCoefficients.maximumIntegralSum;
        controller.coeffs.Kio = newCoefficients.Kio;
        controller.coeffs.Kpo = newCoefficients.Kpo;
    }

    public boolean isBusy() {
        final int curPos = getCurrentPosition();
        return targetPosition - busyRange <= curPos && curPos <= targetPosition + busyRange;
    }
}
