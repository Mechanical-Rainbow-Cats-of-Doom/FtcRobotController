package org.firstinspires.ftc.teamcode.parts;
import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DcMotor;
import androidx.annotation.NonNull;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.ToggleButtonReader;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * lift and arm
 */
public class Lift {
    /**
     * @param map local hardwareMap instance
     * @param telemetry local telemetry instance
     * @param toolGamepad instance of FtcLib GamepadEx
     */
    public Lift(@NonNull HardwareMap map, GamepadEx toolGamepad, Telemetry telemetry) {
        this.liftMotor = map.get(DcMotor.class,"liftMotor");
        this.liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        this.liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        try { Thread.sleep(100); } catch (InterruptedException ignored) {}
        this.liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        assert !liftMotor.isBusy();
        this.armServo = map.get(Servo.class,"armServo");
        this.encoderOffset = liftMotor.getCurrentPosition() * -1;
        this.bottomSensor = map.get(DigitalChannel.class,"bottomSensor");
        this.bottomSensor.setMode(DigitalChannel.Mode.INPUT);
        this.topSensor = map.get(DigitalChannel.class,"topSensor");
        this.topSensor.setMode(DigitalChannel.Mode.INPUT);
        this.telemetry = telemetry;
        this.gamepad = toolGamepad;
        this.rBumpReader = new ToggleButtonReader(toolGamepad, GamepadKeys.Button.RIGHT_BUMPER);
        this.aReader = new ButtonReader(toolGamepad, GamepadKeys.Button.A);
    }

    private final Telemetry telemetry;
    private final DcMotor liftMotor;
    private final Servo armServo;
    private final DigitalChannel bottomSensor;
    private final DigitalChannel topSensor;
    private final GamepadEx gamepad;
    private final ToggleButtonReader rBumpReader;
    private final ButtonReader aReader;
    private final double encoderOffset;
    private double curPos = 0;
    private boolean running = false;
    private boolean first = true;

    public void update() {
        final double stickValue = gamepad.getLeftY();
        telemetry.addData("left stick",stickValue);
        telemetry.addData("lift motor power", liftMotor.getPower());
        telemetry.addData("lift motor state",liftMotor.getMode());
        telemetry.addData("bottomSensor",curPos <= 10);
        telemetry.addData("motor encoder",curPos);
        telemetry.addData("topSensor", topSensor.getState());
        telemetry.addData("armServo",armServo.getPosition());
        if (!liftMotor.isBusy()) {
            if (running) {
                liftMotor.setPower(0);
                liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                running = false;
            }
            aReader.readValue();
            if (aReader.wasJustReleased()) {
                first = true;
                curPos = 10;
            }
            if ((stickValue >= 0.05 && !topSensor.getState()) || (stickValue <= -0.05 && curPos >= 10)) {
                liftMotor.setPower(stickValue);
            } else {
                if (curPos <= 10 && first) {
                    liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                    liftMotor.setTargetPosition(0);
                    liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    liftMotor.setPower(1);
                    running = true;
                    first = false;
                } else {
                    if (topSensor.getState()) {
                        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    } else if (curPos >= 10) {
                        first = true;
                    }
                    liftMotor.setPower(0);
                }
            }
        }
        curPos = liftMotor.getCurrentPosition() + encoderOffset;
        arm();
    }

    private void arm() {
        if (curPos >= 810) { // completely away from intake interference
            rBumpReader.readValue();
            if (rBumpReader.getState()) {
                armServo.setPosition(0.07); // dump pos
            } else {
                armServo.setPosition(0.76); // lift pos
            }
        } else if (curPos >= 50) { // if it is away from load zone but still interfering
            armServo.setPosition(0.85); // in between load and lift pos
        } else { // in load zone
            armServo.setPosition(0.88); // load pos
        }
    }
}