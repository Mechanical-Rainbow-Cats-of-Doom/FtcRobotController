package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.core.robot.tools.impl.CyclerArm;
import org.firstinspires.ftc.teamcode.core.robot.util.EncoderNames;
import org.firstinspires.ftc.teamcode.core.robot.util.PIDServo;
import org.firstinspires.ftc.teamcode.roadrunner.util.Encoder;

@TeleOp
@Config
public class PIDTuner extends LinearOpMode {
    public static int topPos = 0;
    public static int bottomPos = 0;
    public static boolean bothpostop = false;
    @Override
    public void runOpMode() throws InterruptedException {
        final CyclerArm cyclerArm = new CyclerArm(hardwareMap, telemetry);
        waitForStart();
        while (opModeIsActive()) {
            if (bothpostop) bottomPos = topPos;
            cyclerArm.setTopTargetPosition(topPos);
            cyclerArm.setBottomTargetPosition(bottomPos);
            cyclerArm.debugUpdate();
        }
    }
}
