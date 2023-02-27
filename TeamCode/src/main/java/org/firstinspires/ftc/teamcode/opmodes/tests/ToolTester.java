package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.core.robot.tools.impl.auto.AutoTools;
import org.firstinspires.ftc.teamcode.core.robot.tools.impl.auto.AutoTurret;
import org.firstinspires.ftc.teamcode.core.robot.util.ZeroMotorEncoder;

import java.util.Timer;

@Config
@TeleOp
public class ToolTester extends LinearOpMode {
    public static double turretPos = 0;
    public static AutoTurret.Units unit = AutoTurret.Units.DEGREES;
    public static AutoTools.Position position = AutoTools.Position.GROUND_TARGET_NODUMP;
    public static int liftPos = AutoTools.Position.GROUND_TARGET_NODUMP.liftPos;
    public static int armPos = AutoTools.Position.GROUND_TARGET_NODUMP.armPos;
    public static int cyclingPos = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        final DcMotor liftMotor = hardwareMap.get(DcMotor.class, "lift");
        final DcMotor armMotor = hardwareMap.get(DcMotor.class, "arm");
        final DcMotor cyclingMotor = hardwareMap.get(DcMotor.class, "cycler");
        ZeroMotorEncoder.zero(liftMotor, armMotor, cyclingMotor);
        final MultipleTelemetry telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), super.telemetry);
        final AutoTurret turret = new AutoTurret(hardwareMap, 0);
        waitForStart();
        while (opModeIsActive()) {
            liftMotor.setTargetPosition(liftPos);
            armMotor.setTargetPosition(armPos);
            cyclingMotor.setTargetPosition(cyclingPos);
            turret.setPos(turretPos, unit);
            telemetry.addData("turret pos", turret.getPos(unit));
            telemetry.addData("lift pos", liftMotor.getCurrentPosition());
            telemetry.addData("arm pos", armMotor.getCurrentPosition());
            telemetry.addData("cycling pos", cyclingMotor.getCurrentPosition());
            telemetry.update();
        }
        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        cyclingMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turret.cleanup();
    }
}
