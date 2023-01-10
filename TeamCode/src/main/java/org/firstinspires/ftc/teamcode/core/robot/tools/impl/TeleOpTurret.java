package org.firstinspires.ftc.teamcode.core.robot.tools.impl;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.core.robot.util.ZeroMotorEncoder;

import androidx.annotation.NonNull;

public class TeleOpTurret extends AutoTurret {
    private final GamepadEx gamepad;

    @Override
    void initMotors() {
        ZeroMotorEncoder.zero(motor, DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /**
     * Only run after init, robot crashes otherwise
     */
    public TeleOpTurret(HardwareMap hardwareMap, GamepadEx toolGamepad) {
        super(hardwareMap);
        this.gamepad = toolGamepad;
    }

    public void update() {
        TeleOpLift.runBoundedTool(motor, (int)Math.round(minRot*ticksperdeg), (int)Math.round(maxRot*ticksperdeg), gamepad.getLeftX(), false);
    }
}
