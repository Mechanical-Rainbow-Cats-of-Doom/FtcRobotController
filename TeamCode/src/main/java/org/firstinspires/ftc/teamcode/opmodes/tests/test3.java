package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.core.robot.tools.headless.AutoCarousel;
import org.firstinspires.ftc.teamcode.core.robot.tools.headless.AutoLift;
import org.firstinspires.ftc.teamcode.core.robot.vision.old.robot2.FiducialDetector;
import org.firstinspires.ftc.teamcode.core.thread.old.EventThread;
import org.firstinspires.ftc.teamcode.core.thread.old.types.impl.RunEveryTimedEvent;
import org.firstinspires.ftc.teamcode.core.thread.old.types.impl.TimedEvent;
import org.firstinspires.ftc.teamcode.opmodes.auto.CVAuto;
import org.firstinspires.ftc.teamcode.roadrunner.util.Encoder;

@TeleOp
@Config
public class test3 extends LinearOpMode {
    private final EventThread eventThread = new EventThread(() -> !isStopRequested());

    public static double liftMotorPower = 0.1;
    @Override
    public void runOpMode() {
        hardwareMap.get(DcMotor.class,"liftMotor").setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        final AutoCarousel carousel =  new AutoCarousel(hardwareMap);
        final DistanceSensor sensor = hardwareMap.get(DistanceSensor.class, "intakeSensor");
        final Encoder leftEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "intake"));
        final Encoder rightEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "spinner"));
        rightEncoder.setDirection(Encoder.Direction.REVERSE);
        FiducialDetector detector = new FiducialDetector(hardwareMap, "webcam", true, CVAuto.zeroOrOneRedorBlue == 0);
        final Encoder frontEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "backEncoder"));
        final DcMotor one = hardwareMap.get(DcMotorEx.class, "front left wheel");
        final DcMotor two = hardwareMap.get(DcMotorEx.class, "front right wheel");
        final DcMotor three = hardwareMap.get(DcMotorEx.class, "back right wheel");
        final DcMotor four = hardwareMap.get(DcMotorEx.class, "back left wheel");
        four.setDirection(DcMotorSimple.Direction.REVERSE);
        two.setDirection(DcMotorSimple.Direction.REVERSE);
        final DcMotor[] motors = {one, two, three, four};
        final GamepadEx gamepadEx = new GamepadEx(gamepad1);
        final AutoLift lift = new AutoLift(eventThread, hardwareMap);
        waitForStart();
        eventThread.start();
        eventThread.addEvent(new RunEveryTimedEvent(() -> lift.setPosition(AutoLift.Positions.TOP), 6000));
        eventThread.addEvent(new TimedEvent(detector::run, 500));
        while (opModeIsActive()) {
            lift.update();
            final double stick = gamepadEx.getLeftY();
            for (DcMotor motor : motors) {
                motor.setPower(stick);
            }
            telemetry.addData("distance", sensor.getDistance(DistanceUnit.MM));
            telemetry.addData("leftEncoder",leftEncoder.getCurrentPosition());
            telemetry.addData("rightEncoder",rightEncoder.getCurrentPosition());
            telemetry.addData("frontEncoder",frontEncoder.getCurrentPosition());
            telemetry.update();
        }
    }
}
