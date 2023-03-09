package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.core.robot.tools.impl.CyclerArm;

@TeleOp
@Config
public class CyclerArmTuner extends LinearOpMode {
    public static boolean extended;
    @Override
    public void runOpMode() throws InterruptedException {
        final CyclerArm cyclerArm = new CyclerArm(hardwareMap, telemetry, this);
        waitForStart();
        while (opModeIsActive()) {
            cyclerArm.setExtended(extended);
            cyclerArm.update();
        }
    }
}
