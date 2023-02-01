package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.core.robot.util.SewerOodoo;

@Autonomous
public class SewerOodooTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        SewerOodoo odo = new SewerOodoo();
        final BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        imu.initialize(parameters);
        odo.zAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;

        waitForStart();

        odo.SetAxisMovement();
        odo.ZeroEncoders();
        odo.SetPresetMovement(5,1,0,1, odo.zAngle);
        while(odo.MoveToLocation()){
            odo.SetRotation(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
        }

    }
}
