package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.core.robot.tools.impl.auto.AutoTools;
import org.firstinspires.ftc.teamcode.core.robot.tools.impl.auto.AutoTurret;

import java.util.Timer;

@Config
@TeleOp
public class ToolTester extends LinearOpMode {
    public static double turretPos = 0;
    public static AutoTurret.Units unit = AutoTurret.Units.DEGREES;
    public static AutoTools.Position position = AutoTools.Position.GROUND_TARGET_NODUMP;
    @Override
    public void runOpMode() throws InterruptedException {
        final MultipleTelemetry telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), super.telemetry);
        final AutoTurret turret = new AutoTurret(hardwareMap);
        final AutoTools tools = new AutoTools(hardwareMap, new Timer(), turret);
        waitForStart();
        while (opModeIsActive()) {
            tools.setPosition(position);
            tools.update();
            turret.setPos(turretPos, unit);
            telemetry.addData("turret pos", turret.getPos(unit));
            telemetry.update();
        }
        tools.cleanup();
    }
}
