package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class ServoConfigurationWithMotor extends ServoConfiguration {
    @Override
    public void runOpMode() {
        super.withMotor = true;
        super.runOpMode();
    }
}