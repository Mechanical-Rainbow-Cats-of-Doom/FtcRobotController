package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Autonomous
@Disabled
public class RedAutoClose extends BlueAutoClose {
    public RedAutoClose() {
        this.isRed = true;
    }
}
