package org.firstinspires.ftc.teamcode.core.robot.tools.impl;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.core.robot.util.ZeroMotorEncoder;

import androidx.annotation.NonNull;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

public class AutoTurret {
    public enum Position {
        FRONT(0),
        FRONTRIGHT(45),
        RIGHT(90),
        BACKRIGHT(135),
        BACK(180),
        BACKLEFT(225),
        LEFT(270),
        FRONTLEFT(315);

        final double turretPos;
        Position(double turretPos) {
            this.turretPos = turretPos * ticksperdeg;
        }
    }
    public static double maxRot = 110;
    public static double minRot = -200;

    final DcMotor motor;
    public static final double tpr = (((1+(46D/17))) * (1+(46D/11))) * 28 * 5; // 5 for gear
    public static final double ticksperdeg = tpr / 360;

    void initMotors() {
        ZeroMotorEncoder.zero(motor);
    }

    /**
     * Only run after init, robot crashes otherwise
     */
    public AutoTurret(@NonNull HardwareMap hardwareMap) {
        this.motor = hardwareMap.get(DcMotor.class, "turret");
        motor.setZeroPowerBehavior(BRAKE);
        initMotors();
    }

    /**
     * sets position of autoTurret in degrees, goes around if it would result in going through start pos
     * @param pos MUST BE BETWEEN {@value maxRot} & {@value minRot} OR THE ROBOT WILL KILL ITSELF
     */
    public void setPosDeg(double pos) {
        assert pos <= maxRot && pos >= minRot;
        motor.setTargetPosition((int) Math.round(pos * ticksperdeg));
    }

    public boolean isMoving() {
        return motor.isBusy();
    }

    /**
     * don't call too often, relatively resource intensive
     * @return current pos in degrees
     */
    public double getPosDeg() {
        return motor.getCurrentPosition() / ticksperdeg;
    }
}