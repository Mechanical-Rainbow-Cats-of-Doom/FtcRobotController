package org.firstinspires.ftc.teamcode.opmodes.drive;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.core.robot.ControllerMovement;
import org.firstinspires.ftc.teamcode.core.robot.distance.FourDistanceSensors;
import org.firstinspires.ftc.teamcode.core.robot.tools.driveop.ControllerCarousel;
import org.firstinspires.ftc.teamcode.core.robot.tools.driveop.ControllerGrabber;
import org.firstinspires.ftc.teamcode.core.robot.tools.driveop.ControllerIntake;
import org.firstinspires.ftc.teamcode.core.robot.tools.driveop.ControllerLift;
import org.firstinspires.ftc.teamcode.core.thread.old.EventThread;

@TeleOp
public class distanceSensorTest extends LinearOpMode {
    @Override
    public void runOpMode(){
        MultipleTelemetry goodTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        FourDistanceSensors sensors = new FourDistanceSensors(hardwareMap);
        waitForStart();
        sensors.init(); // front right back left
        while (opModeIsActive()) {
            for (int i = 0; i < 4; i++) {
                telemetry.addData("distance " + i, sensors.getDistance(i));
                telemetry.addData("isObject " + i, sensors.isObject(i, 5));
            }
            telemetry.update();
        }
    }
}
