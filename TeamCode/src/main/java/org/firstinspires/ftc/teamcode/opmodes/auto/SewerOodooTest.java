package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.*;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.core.robot.util.EncoderNames;
import org.firstinspires.ftc.teamcode.core.robot.util.SewerOodoo;
import org.firstinspires.ftc.teamcode.roadrunner.util.Encoder;

@Autonomous
@Disabled
public class SewerOodooTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        SewerOodoo odo = new SewerOodoo(hardwareMap, telemetry);
        ElapsedTime timer = new ElapsedTime();

//        final BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "imu");
//        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
//        imu.initialize(parameters);
//        odo.zAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;

        waitForStart();

        odo.SetAxisMovement();
        odo.ZeroEncoders();
        try {
            odo.SetPresetMovement(1,0.2,0,1, 0);
            telemetry.addLine("route good");
        } catch (Exception e){
            telemetry.addLine("route bad");
        }
        telemetry.update();


        while(!odo.MoveToLocation() && opModeIsActive()){
            odo.SetRotation(0);
            telemetry.addLine("moving");
            telemetry.addData("goal", odo.drivePreset);
            telemetry.addData("current drive", odo.trueDrive);
            telemetry.update();
        }
//        telemetry.addLine("works 5");
//        telemetry.update();
//        odo.testDrive();
//        telemetry.addLine("works 6");
//        telemetry.update();
//        while (opModeIsActive()){
//            telemetry.addLine("works 99");
//            telemetry.update();
//        }
//        odo.TryTelemetry();
    }
}
