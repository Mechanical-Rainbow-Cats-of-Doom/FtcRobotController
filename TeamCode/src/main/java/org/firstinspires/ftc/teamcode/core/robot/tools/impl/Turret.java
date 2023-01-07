package org.firstinspires.ftc.teamcode.core.robot.tools.impl;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.core.robot.util.ZeroMotorEncoder;
import androidx.annotation.NonNull;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

@SuppressWarnings("BusyWait")
@Config
public class Turret {
    public static double startOffset = 45;
    public static double maxRot = 110;
    public static double minRot = -200;

    private final DcMotor motor;
    private final double tpr = (((1+(46D/17))) * (1+(46D/11))) * 28 * 5; // 5 for gear
    private final double ticksperdeg = tpr / 360;

    /**
     * Only run after init, robot crashes otherwise
     * @param motor turret motor
     */
    public Turret(@NonNull DcMotor motor) {
        this.motor = motor;
        motor.setZeroPowerBehavior(BRAKE);
        ZeroMotorEncoder.zero(motor);
        motor.setTargetPosition((int) Math.round(startOffset * ticksperdeg));
        Thread thread = new Thread(() -> {
            while (motor.isBusy()){
                try {
                    Thread.sleep(150);
                } catch (InterruptedException ignored) {}
            }
            ZeroMotorEncoder.zero(motor);
        });
        thread.setPriority(Thread.MIN_PRIORITY);
        thread.start();
    }

    /**
     * sets position of turret in degrees, goes around if it would result in going through start pos
     * @param pos MUST BE BETWEEN {@value maxRot} & {@value minRot} OR THE ROBOT WILL KILL ITSELF
     */
    public void setPosDeg(double pos) {
        assert pos <= maxRot && pos >= minRot;
        motor.setTargetPosition((int) Math.round(pos * ticksperdeg));
    }

    /**
     * don't call too often, relatively resource intensive
     * @return current pos in degrees
     */
    public double getPosDeg() {
        return motor.getCurrentPosition() / ticksperdeg;
    }
}
