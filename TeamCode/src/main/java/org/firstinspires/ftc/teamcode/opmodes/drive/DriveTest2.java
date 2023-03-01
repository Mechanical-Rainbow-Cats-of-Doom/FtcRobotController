package org.firstinspires.ftc.teamcode.opmodes.drive;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.core.movement.impl.StrafedMovementImpl;
@Disabled
public class DriveTest2 extends LinearOpMode {
    @Override
    public void runOpMode() {
        StrafedMovementImpl drive = new StrafedMovementImpl(hardwareMap);
        final GamepadEx moveGamepad = new GamepadEx(gamepad1);
        waitForStart();
        while (opModeIsActive()) {
            drive.driveDRS(moveGamepad.getLeftY(), moveGamepad.getLeftX(), moveGamepad.getRightX());
        }
    }
}
