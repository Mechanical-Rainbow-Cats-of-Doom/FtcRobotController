package org.firstinspires.ftc.teamcode.opmodes.drive;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.core.robot.drive.FunnyControllerMovement;

@TeleOp
public class FunnyDrive extends NormalDrive {
    @Override
    FunnyControllerMovement createDrive() {
        return new FunnyControllerMovement(hardwareMap, moveGamepad);
    }
}
