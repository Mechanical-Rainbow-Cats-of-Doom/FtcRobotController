package org.firstinspires.ftc.teamcode.core.robot.tools.impl;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.hardware.DcMotor;

import androidx.annotation.NonNull;

public class TeleOpTurret extends AutoTurret {
    private final GamepadEx gamepad;
    /**
     * Only run after init, robot crashes otherwise
     *
     * @param motor autoTurret motor
     */
    public TeleOpTurret(@NonNull DcMotor motor, GamepadEx toolGamepad) {
        super(motor);
        this.gamepad = toolGamepad;
    }

    public void update() {
        TeleOpLift.runBoundedTool(motor, (int)Math.round(minRot*ticksperdeg), (int)Math.round(maxRot*ticksperdeg), gamepad.getLeftX());
    }
}
