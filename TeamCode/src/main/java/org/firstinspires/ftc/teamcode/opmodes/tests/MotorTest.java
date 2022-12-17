package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.core.movement.impl.StrafedMovementImpl;

@TeleOp
public class MotorTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        StrafedMovementImpl movement = new StrafedMovementImpl(hardwareMap);

        waitForStart();

        while(!isStopRequested()) {
            int rL = 0, rR = 0, fL = 0, fR = 0;
            if(gamepad1.a) {
                rR = 1;
            }
            if(gamepad1.dpad_down) {
                rL = 1;
            }
            if(gamepad1.left_bumper) {
                fL = 1;
            }
            if(gamepad1.right_bumper) {
                fR = 1;
            }
            movement.drivePower(fL, fR, rR, rL);
        }
    }
}
