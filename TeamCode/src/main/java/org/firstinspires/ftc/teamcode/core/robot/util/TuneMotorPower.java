package org.firstinspires.ftc.teamcode.core.robot.util;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
@Config
public class TuneMotorPower extends LinearOpMode {
    public static String motorName = "arm";
    public static double power = 0;
    @Override
    public void runOpMode(){
        waitForStart();
        while (opModeIsActive()) {
            hardwareMap.get(DcMotor.class, motorName).setPower(power);
        }
    }
}
