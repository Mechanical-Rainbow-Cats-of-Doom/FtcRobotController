package org.firstinspires.ftc.teamcode.core.robot.tools.impl;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.PIDEx;
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficientsEx;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.teamcode.core.robot.util.EncoderNames;
import org.firstinspires.ftc.teamcode.roadrunner.util.Encoder;

@Config
public class CyclerArm {
    public static double Kp = 0.6, Ki = 1.2, Kd = 0.075, integralSumMax = 1, stability_thresh = 0, lowPassGain = 0; // turn this into a class and make 2 of them
    public static boolean updateController = false;
    private final double targetPosition = 0;
    public final Encoder topEncoder, bottomEncoder;
    public final CRServo topServo, bottomServo;
    public PIDEx controller;
    private void updateController() {
        controller = new PIDEx(new PIDCoefficientsEx(Kp, Ki, Kd, integralSumMax, stability_thresh, lowPassGain));
    }
    public CyclerArm(HardwareMap hardwareMap, Telemetry telemetry) {
        topEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, EncoderNames.topArm));
        bottomEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, EncoderNames.bottomArm));
        topServo = hardwareMap.get(CRServo.class, "top");
        bottomServo = hardwareMap.get(CRServo.class, "bottom");
        updateController();
    }

    public void update(boolean debug) {
        if (debug && updateController) {
            updateController = false;
            updateController();
        }

    }

    public void update() {
        update(false);
    }
}
