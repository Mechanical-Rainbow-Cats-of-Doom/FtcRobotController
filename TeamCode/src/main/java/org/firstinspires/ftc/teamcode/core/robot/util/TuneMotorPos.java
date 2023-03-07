package org.firstinspires.ftc.teamcode.core.robot.util;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
@Config
public class TuneMotorPos extends LinearOpMode {
    public static String motorName = "arm";
    public static int pos = 0;
    public static double power = 0.25D;
    @Override
    public void runOpMode(){
        final MultipleTelemetry telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), this.telemetry);
        waitForStart();
        while (opModeIsActive()) {
            final DcMotor motor = hardwareMap.get(DcMotor.class, motorName);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor.setPower(power);
            motor.setTargetPosition(pos);
            telemetry.addData("motorpos", motor.getTargetPosition());
            telemetry.update();
        }
    }
}
