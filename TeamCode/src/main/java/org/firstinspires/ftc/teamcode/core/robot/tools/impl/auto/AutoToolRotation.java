package org.firstinspires.ftc.teamcode.core.robot.tools.impl.auto;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.core.robot.util.ZeroMotorEncoder;

import androidx.annotation.NonNull;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

public class AutoToolRotation {
    public enum Rotation {
        FRONT(0),
        FRONTRIGHT(45),
        RIGHT(90),
        BACKRIGHT(135),
        BACK(180),
        BACKLEFT(225),
        LEFT(270),
        FRONTLEFT(315);

        final double motorPos;
        Rotation(double motorPos) {
            this.motorPos = motorPos * ticksperdeg;
        }
    }

    public static double maxRot = 1100;
    public static double minRot = -2000;

    protected final DcMotor motor;
    // thanks ethan for the unlabeled magic numbers!
    // well i assume it was you but you were on logans computer or something
    public static final double tpr = (((1+(46D/17))) * (1+(46D/11))) * 28 * 5; // 5 for gear
    public static final double ticksperdeg = tpr / 360;

    protected void initMotors() {
        ZeroMotorEncoder.zero(motor);
    }

    /**
     * Only run after init, robot crashes otherwise
     */
    public AutoToolRotation(@NonNull HardwareMap hardwareMap) {
        this.motor = hardwareMap.get(DcMotor.class, "turret");
        motor.setZeroPowerBehavior(BRAKE);
        initMotors();
    }

    /**
     * sets the rotation of the tool in degrees, goes around if it would result in going through start pos
     * @param pos MUST BE BETWEEN {@value maxRot} & {@value minRot} OR THE ROBOT WILL KILL ITSELF
     */
    public void setPos(double pos, boolean isDeg) {
        assert pos <= maxRot + 1 && pos >= minRot -1;
        if (isDeg) {
            pos *= ticksperdeg;
        }
        motor.setTargetPosition((int) Math.round(pos));
    }

    public boolean isMoving() {
        return motor.isBusy();
    }

    /**
     * don't call too often, relatively resource intensive
     * @return current pos in degrees
     */
    public double getPos(boolean deg) {
        return deg ? motor.getCurrentPosition() / ticksperdeg : motor.getCurrentPosition();
    }
}