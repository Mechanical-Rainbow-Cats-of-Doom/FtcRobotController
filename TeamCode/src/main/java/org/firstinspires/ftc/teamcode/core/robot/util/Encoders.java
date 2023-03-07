package org.firstinspires.ftc.teamcode.core.robot.util;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.LinkedHashMap;
import java.util.Map;

@TeleOp
public class Encoders extends LinearOpMode {

    @Override
    public void runOpMode() {
        final MultipleTelemetry telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), this.telemetry);
        final LinkedHashMap<String, DcMotor> motors = new LinkedHashMap<String, DcMotor>() {{
            put("rightRear", null);
            put("rightFront", null);
            put("leftFront", null);
            put("leftRear", null);
            put("lift", null);
            put("arm", null);
            put("cycler", null);
            put("turret", null);
        }};
        motors.replaceAll((motorName, v) -> hardwareMap.get(DcMotor.class, motorName));
        waitForStart();
        while (opModeIsActive()) {
            for (Map.Entry<String, DcMotor> entry : motors.entrySet()) {
                telemetry.addData(entry.getKey() + " pos", entry.getValue().getCurrentPosition());
            }
            telemetry.update();
        }
    }
}
