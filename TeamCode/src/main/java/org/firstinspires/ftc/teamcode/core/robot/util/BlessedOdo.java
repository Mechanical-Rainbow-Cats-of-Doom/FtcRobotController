package org.firstinspires.ftc.teamcode.core.robot.util;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.roadrunner.util.Encoder;

@Config
public class BlessedOdo {
    Encoder leftEncoder, rightEncoder, frontEncoder;
    private DcMotorEx leftFront, leftRear, rightRear, rightFront;
    private BNO055IMU imu;

    public BlessedOdo(HardwareMap hardwareMap){
//        this.hardwareMap = hardwareMap;
        leftEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, EncoderNames.leftEncoder));
        rightEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, EncoderNames.rightEncoder));
        frontEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, EncoderNames.frontEncoder));

        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
        rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);
    }
    private boolean drove;
    private boolean strafed;
    private boolean rotated;
    private int forwardDistance = 0;
    private int driveReset = 0;
    public static double driveSpeedMultiplier = 0.5;
    private int sidewaysDistance = 0;
    private int strafeReset = 0;
    public static double strafeSpeedMultiplier = 0.5;
    private double rotationDistance = 0;
    private double rotateReset = 0;
    public static double rotationSpeedMultiplier = 0.5;
    public static double accelerationAngle = 2;
    // https://www.desmos.com/calculator/jjdwtdu83r

    public static int leftRearDirection = 1;
    public static int leftFrontDirection = 1;
    public static int rightRearDirection = 1;
    public static int rightFrontDirection = 1;

    public void ResetEncoders(){
        this.driveReset = Math.round((rightEncoder.getCurrentPosition() + leftEncoder.getCurrentPosition())/2);
        this.strafeReset = leftEncoder.getCurrentPosition();
        this.rotateReset = imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
    }

    public void Path(int forward, int sideways, double rotation){
        ResetEncoders();

        this.rotationDistance = rotation;

        if (forward == 0) {
            this.forwardDistance = 1;
        } else {
            this.forwardDistance = forward;
        }

        if (sideways == 0) {
            this.sidewaysDistance = 1;
        } else {
            this.sidewaysDistance = sideways;
        }
    }

    public void SetMotors(double drive, double strafe, double rotate) {
        this.leftFront.setPower((-drive + strafe + rotate)* -leftFrontDirection);
        this.leftRear.setPower((-drive - strafe + rotate)* -leftRearDirection);
        this.rightFront.setPower((drive + strafe + rotate)* rightFrontDirection);
        this.rightRear.setPower((drive - strafe + rotate)* rightRearDirection);
    }

    public void Drive() {

        int drive = Math.round((rightEncoder.getCurrentPosition() + leftEncoder.getCurrentPosition())/2) - this.driveReset;
        int strafe = leftEncoder.getCurrentPosition() - this.strafeReset;
        double rotation = imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle - this.rotateReset;

        double driveMovement;
        double strafeMovement;
        double rotationMovement;

        if (Math.abs(this.forwardDistance - drive) >= 100) {
            driveMovement = Math.signum(this.forwardDistance - drive) * Math.max(0.15, driveSpeedMultiplier);
        } else {
            this.drove = true;
            driveMovement = 0;
        }

        if (Math.abs(this.sidewaysDistance - strafe) >= 50) {
            strafeMovement = Math.signum(this.sidewaysDistance - strafe) * Math.max(0.15, strafeSpeedMultiplier);
        } else {
            this.strafed = true;
            strafeMovement = 0;
        }

        if ((Math.abs(this.rotationDistance - rotation) >= 2)) {
            rotationMovement = Math.signum(this.rotationDistance - rotation) * Math.max(0.15, rotationSpeedMultiplier);
        } else {
            this.rotated = true;
            rotationMovement = 0;
        }

        SetMotors(driveMovement,strafeMovement,rotationMovement);
    }

    public void Drive2() {

        int drive = Math.round((rightEncoder.getCurrentPosition() + leftEncoder.getCurrentPosition())/2) - this.driveReset;
        int strafe = leftEncoder.getCurrentPosition() - this.strafeReset;
        double rotation = imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle - this.rotateReset;

        double driveMovement;
        double strafeMovement;
        double rotationMovement;

        if (Math.abs(this.forwardDistance - drive) >= 100) {
            driveMovement = Math.signum(this.forwardDistance - drive) * Math.signum(this.forwardDistance)*Math.max(0.15, driveSpeedMultiplier * (Math.abs(this.forwardDistance-drive)/this.forwardDistance)*(this.forwardDistance/this.accelerationAngle));
        } else {
            this.drove = true;
            driveMovement = 0;
        }

        if (Math.abs(this.sidewaysDistance - strafe) >= 50) {
            strafeMovement = Math.signum(this.sidewaysDistance - strafe) * Math.signum(this.sidewaysDistance)*Math.max(0.15, strafeSpeedMultiplier*(Math.abs(this.sidewaysDistance-strafe)/this.sidewaysDistance)*(this.sidewaysDistance/this.accelerationAngle));
        } else {
            this.strafed = true;
            strafeMovement = 0;
        }

        if ((Math.abs(this.rotationDistance - rotation) >= 2)) {
            rotationMovement = Math.signum(this.rotationDistance - rotation) * Math.signum(this.rotationDistance)*Math.max(0.15, rotationSpeedMultiplier*(Math.abs(this.rotationDistance-rotation)/this.rotationDistance)*(this.rotationDistance/this.accelerationAngle));
        } else {
            this.rotated = true;
            rotationMovement = 0;
        }

        SetMotors(driveMovement,strafeMovement,rotationMovement);
    }

    public boolean Moving(){
        boolean moving;

        if (this.drove && this.rotated && this.strafed) {
            drove = false;
            rotated = false;
            strafed = false;
            leftFront.setPower(-0.01);
            rightFront.setPower(-0.01);
            rightRear.setPower(-0.01);
            leftRear.setPower(-0.01);
            moving = false;
        } else {
            drove = false;
            rotated = false;
            strafed = false;
            moving = true;
        }

        return (moving);
    }
}
