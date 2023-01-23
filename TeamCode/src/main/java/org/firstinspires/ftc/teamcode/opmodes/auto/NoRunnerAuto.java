package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.core.robot.util.EncoderNames;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.util.Encoder;

@Autonomous
public class NoRunnerAuto extends LinearOpMode {
    public int forwardDistance = 39000;

    final SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

    final Encoder leftEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, EncoderNames.leftEncoder));
    int LEReset = leftEncoder.getCurrentPosition();
    final Encoder rightEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, EncoderNames.rightEncoder));
    int REReset = rightEncoder.getCurrentPosition();
    final Encoder frontEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, EncoderNames.frontEncoder));
    int FEReset = frontEncoder.getCurrentPosition();

    @Override
    public void runOpMode() throws InterruptedException {

//        drive.setWeightedDrivePower();
    }
}
