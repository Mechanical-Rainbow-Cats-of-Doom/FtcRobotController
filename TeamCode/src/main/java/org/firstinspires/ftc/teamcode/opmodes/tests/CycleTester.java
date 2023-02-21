package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.core.robot.tools.impl.Cycler;
import org.firstinspires.ftc.teamcode.core.robot.tools.impl.auto.AutoTools;
import org.firstinspires.ftc.teamcode.core.robot.tools.impl.auto.AutoTurret;

import java.util.Timer;

@Config
@TeleOp
public class CycleTester extends LinearOpMode {
    public static Cycler.Cycles cycleType = Cycler.Cycles.ALLIANCE_LEFT_HIGH;
    public static int howManyCones = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        final MultipleTelemetry telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), super.telemetry);
        final AutoTurret turret = new AutoTurret(hardwareMap, 0);
        final AutoTools tools = new AutoTools(hardwareMap, new Timer(), turret, this, telemetry);
        waitForStart();
        while (opModeIsActive()) {
            if (tools.isCycling()) {
                telemetry.addData("cones dumped", tools.getConesDumped());
            } else if (howManyCones > 0) {
                tools.startCycling(cycleType, howManyCones);
                howManyCones = 0;
            }
            telemetry.update();
        }
        tools.cleanup();
    }
}
