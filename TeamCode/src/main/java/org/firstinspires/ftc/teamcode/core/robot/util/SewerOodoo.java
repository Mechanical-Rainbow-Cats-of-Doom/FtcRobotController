// https://tenor.com/view/we-do-a-medium-amount-of-trolling-we-do-a-lot-of-trolling-troll-trolling-we-do-a-little-trolling-gif-20600937
package org.firstinspires.ftc.teamcode.core.robot.util;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.core.robot.util.EncoderNames;
import org.firstinspires.ftc.teamcode.roadrunner.util.Encoder;

import java.lang.Math;

public class SewerOodoo {
    Encoder leftEncoder2, rightEncoder2, frontEncoder2;
//    HardwareMap hardwareMap;
    Telemetry telemetry;
    private DcMotorEx leftFront, leftRear, rightRear, rightFront;
    public SewerOodoo(HardwareMap hardwareMap, Telemetry telemetry){
//        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        leftEncoder2 = new Encoder(hardwareMap.get(DcMotorEx.class, EncoderNames.leftEncoder));
        rightEncoder2 = new Encoder(hardwareMap.get(DcMotorEx.class, EncoderNames.rightEncoder));
        frontEncoder2 = new Encoder(hardwareMap.get(DcMotorEx.class, EncoderNames.frontEncoder));
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
        rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
    }
//
//    ElapsedTime movementTimer = new ElapsedTime();
//    final Encoder leftEncoder2 = new Encoder(hardwareMap.get(DcMotorEx.class, EncoderNames.leftEncoder));
//    final Encoder rightEncoder2 = new Encoder(hardwareMap.get(DcMotorEx.class, EncoderNames.rightEncoder));
//    final Encoder frontEncoder2 = new Encoder(hardwareMap.get(DcMotorEx.class, EncoderNames.frontEncoder));


    private DigitalChannel switch_;

    double frontLeft;
    double frontRight;
    double backLeft;
    double backRight;
    double rightEncoder;
    double leftEncoder;
    double backEncoder;
    double clearRight = 0;
    double clearLeft = 0;
    double clearBack = 0;
    double clearRotate = 0;
    double fieldLength = 141;
    double robotLength = 17.25;
    double robotWidth = 17.375;
    double countsPerRotation = 360;
    public double trueDrive;

    double trueStrafe;
    double slowIntensity = 10;
    double trueRotate;
    double backRightMultiplier = 1;
    double backLeftMultiplier = 1;
    double frontRightMultiplier = 0.9;
    double frontLeftMultiplier = 0.9;
    double oldZAngel = 0;
    double newZAngle = 0;
    double rotations = 0;
    public double zAngle = 0;
    double presetX = 0;
    double presetY = 0;
    double trueX = 0;
    double trueY = 0;
    double clearDrive = 0;
    double clearStrafe = 0;
    double tau = 6.28318530718;

    double IMUDrive = 0;
    double IMUStrafe = 0;


    double drive;
    double driveSpeedMultiplier = 1;
    int isDrive = 0;

    double strafe;
    double strafeSpeedMultiplier = 1;
    int isStrafe = 0;

    double rotation;
    int isRotate = 0;

    public double drivePreset = 0;
    double strafePreset = 0;
    double rotationPreset = 0;

    boolean isDone = false;

    public void TryTelemetry(){
//        try {
//            telemetry.addData("right encoder", hardwareMap.get(DcMotorEx.class, EncoderNames.rightEncoder));
//        } catch (Exception e){
//            telemetry.addLine("Problem with right encoder");
//        }
//        try {
//            telemetry.addData( "left encoder",hardwareMap.get(DcMotorEx.class, EncoderNames.leftEncoder));
//        } catch (Exception e){
//            telemetry.addLine("Problem with left encoder");
//        }
//        try {
//            telemetry.addData( "front encoder",hardwareMap.get(DcMotorEx.class, EncoderNames.frontEncoder));
//        } catch (Exception e){
//            telemetry.addLine("Problem with front encoder");
//        }
//        try{
//            telemetry.addData("encoder f ", new Encoder(hardwareMap.get(DcMotorEx.class, EncoderNames.frontEncoder)));
//        } catch (Exception e){
//            telemetry.addLine("oops");
//        }
//        try{
//            telemetry.addData("encoder l ", new Encoder(hardwareMap.get(DcMotorEx.class, EncoderNames.leftEncoder)));
//        } catch (Exception e){
//            telemetry.addLine("oops");
//        }
//        try{
//            telemetry.addData("encoder r ", new Encoder(hardwareMap.get(DcMotorEx.class, EncoderNames.rightEncoder)));
//        } catch (Exception e){
//            telemetry.addLine("oops");
//        }
//        new Encoder(hardwareMap.get(DcMotorEx.class, EncoderNames.frontEncoder));
//        final Encoder leftEncoder2 = new Encoder(hardwareMap.get(DcMotorEx.class, EncoderNames.leftEncoder));
//        final Encoder rightEncoder2 = new Encoder(hardwareMap.get(DcMotorEx.class, EncoderNames.rightEncoder));
//        final Encoder frontEncoder2 = new Encoder(hardwareMap.get(DcMotorEx.class, EncoderNames.frontEncoder));
        while(true){
            try {
                telemetry.addData("re", this.rightEncoder2.getCurrentPosition());
            } catch (Exception e){
                telemetry.addLine("re prob");
            }
            try {
                telemetry.addData("le", this.leftEncoder2.getCurrentPosition());
            } catch (Exception e){
                telemetry.addLine("le prob");
            }
            try {
                telemetry.addData("fe", this.frontEncoder2.getCurrentPosition());
            } catch (Exception e){
                telemetry.addLine("fe prob");
            }
            telemetry.update();
        }



    }

    public void SetPresetMovement(double Preset_Drive, double Drive_Speed_Multiplier, double Preset_Strafe, double Strafe_Speed_Multiplier, double Preset_Rotation) {
        drivePreset = trueDrive + Preset_Drive;
        strafePreset = trueStrafe + Preset_Strafe;
        rotationPreset = Preset_Rotation;
        strafeSpeedMultiplier = Strafe_Speed_Multiplier;
        driveSpeedMultiplier = Drive_Speed_Multiplier;

    }

    public double StrafeMovement(double currentStrafe, double strafeGoal) {

        strafe = Math.signum(strafeGoal - currentStrafe) * Math.max(0.15, Math.abs((strafeGoal - currentStrafe) / strafeGoal));
        return (strafe);
    }

    public double DriveMovement(double currentDrive, double driveGoal) {

        drive = Math.signum(driveGoal - currentDrive) * Math.max(0.15, Math.abs((driveGoal - currentDrive) / driveGoal));
        return (drive);
    }

    public double CorrectRotation(double currentRotation, double rotationGoal, double speed) {

        rotation = Math.signum(rotationGoal - currentRotation) * (Math.max(0.2, speed * Math.abs((rotationGoal - currentRotation) / 180)));
        return (rotation);
    }

    public boolean MoveToLocation() {
        this.SetAxisMovement();

        if (drivePreset == 0) {
            drivePreset = 0.0000000000000000000000001;
        }
        if (strafePreset == 0) {
            strafePreset = 0.0000000000000000000000001;
        }

//        telemetry.addLine("works d ");
        telemetry.update();
        if (Math.abs(drivePreset - trueDrive) >= 2) {
            drive = Math.signum(drivePreset - trueDrive) * Math.max(0.2, driveSpeedMultiplier * Math.abs((drivePreset - trueDrive) / drivePreset));
        } else {
            isDrive = 1;
            drive = 0;
        }

//        telemetry.addLine("works s");
        telemetry.update();
        if (Math.abs(strafePreset - trueStrafe) >= 1) {
            strafe = Math.signum(strafePreset - trueStrafe) * Math.max(0.2, strafeSpeedMultiplier * Math.abs((strafePreset - trueStrafe) / strafePreset));
        } else {
            isStrafe = 1;
            strafe = 0;
        }

//        telemetry.addLine("works r");
        telemetry.update();
        if ((Math.abs(zAngle - rotationPreset) >= 2)) {
            rotation = Math.signum(rotationPreset - zAngle) * (Math.max(0.2, Math.abs((rotationPreset - zAngle) / 180)));
        } else {
            isRotate = 1;
            rotation = 0;
        }


        this.SetMotors(drive, strafe, rotation);
//        telemetry.addLine("works 1");
//        telemetry.update();
//        this.Drive();
//        telemetry.addLine("works 2");
//        telemetry.update();

        if ((isDrive == 1) & (isRotate == 1) & (isStrafe == 1)) {
            isDrive = 0;
            isRotate = 0;
            isStrafe = 0;
            leftFront.setPower(-0.01);
            rightFront.setPower(-0.01);
            rightRear.setPower(-0.01);
            leftRear.setPower(-0.01);
            isDone = true;
        } else {
            isDrive = 0;
            isRotate = 0;
            isStrafe = 0;
            isDone = false;
        }

//        telemetry.addLine("works 3");
//        telemetry.update();
        return (isDone);
    }

    public void SetTrueAxis() {
        trueX = trueStrafe * Math.cos((trueRotate)) + trueDrive * (Math.cos(trueRotate + (Math.PI / 2))) + presetX;
        trueY = trueStrafe * Math.sin((trueRotate)) + trueDrive * (Math.sin(trueRotate + (Math.PI / 2))) + presetY;
    }

    public void SetPresetAxis() {
        presetX = trueX;
        presetY = trueY;
        clearDrive = (rightEncoder + leftEncoder) / 2;
        clearStrafe = backEncoder - (rightEncoder - leftEncoder) / 2;
    }


    public void SetAxisMovement() {
        rightEncoder = rightEncoder2.getCurrentPosition() / 360 * 1.173150521 - clearRight;
        leftEncoder = rightEncoder2.getCurrentPosition() / 360 * 1.178221633 - clearLeft;
        backEncoder = frontEncoder2.getCurrentPosition() / 360 * 1.17584979 - clearBack;
        trueDrive = ((rightEncoder + leftEncoder) / 2) - clearDrive;
        trueStrafe = (backEncoder - (rightEncoder - leftEncoder) / 2) - clearStrafe;
        trueRotate = (((rightEncoder - leftEncoder) / 2) * 0.12877427457 /* <---- see comment below*/) - clearRotate;
            /*
            the number 0.12877427457 was calculated through taking several measurements with the im
            and with the rotate encoders, then I divided the imu measurement by the encoder measurement
            and got the average of all my attempts giving me a multiplier to go from encoders counts
            to degrees, 7.3305, then I multiplied that by the rotation to radians multiplier, pi/180,
            then I ran the robot and rotated 360 degrees several times to get measurements with the
            new rotate values then I divided tau by the new rotation measurements and got the average
            of all my attempts to get the multiplier to go from my new measurements to true tau,
            1.00651012115, then I multiplied my new multiplier by my old multiplier to get my final
            multiplier, 0.12877427457
             */

        if (trueRotate >= tau) {
            clearRotate += tau;
        }
        if (trueRotate < 0) {
            clearRotate -= tau;
        }

    }

    public void ForwardAndBackward(double drivePreset) {
        rightFront.setPower(frontRightMultiplier * Math.signum(drivePreset - trueDrive) * Math.max(0.15, Math.abs((drivePreset - trueDrive) / drivePreset)));
        leftFront.setPower(frontLeftMultiplier * Math.signum(drivePreset - trueDrive) * Math.max(0.15, Math.abs((drivePreset - trueDrive) / drivePreset)));
        leftRear.setPower(-1 * backLeftMultiplier * Math.signum(drivePreset - trueDrive) * Math.max(0.15, Math.abs((drivePreset - trueDrive) / drivePreset)));
        rightRear.setPower(-1*backRightMultiplier * Math.signum(drivePreset - trueDrive) * Math.max(0.15, Math.abs((drivePreset - trueDrive) / drivePreset)));

    }

    public void SetRotation(double imuZAngle) {
        oldZAngel = newZAngle;
        newZAngle = imuZAngle;
        if ((Math.signum(newZAngle) != Math.signum(oldZAngel)) & Math.abs(newZAngle) > 50) {
            rotations += Math.signum(oldZAngel);
        }
        zAngle = (newZAngle + rotations * 360);
    }

    public void LeftAndRight(double drivePreset) {
        rightFront.setPower(-1 * frontRightMultiplier * Math.signum(drivePreset - trueStrafe) * Math.max(0.15, Math.abs((drivePreset - trueStrafe) / drivePreset)));
        leftFront.setPower(frontLeftMultiplier * Math.signum(drivePreset - trueStrafe) * Math.max(0.15, Math.abs((drivePreset - trueStrafe) / drivePreset)));
        leftRear.setPower(backLeftMultiplier * Math.signum(drivePreset - trueStrafe) * Math.max(0.15, Math.abs((drivePreset - trueStrafe) / drivePreset)));
        rightRear.setPower(-1*backRightMultiplier * Math.signum(drivePreset - trueStrafe) * Math.max(0.15, Math.abs((drivePreset - trueStrafe) / drivePreset)));

    }


    public void ZeroEncoders() {
        clearRight = rightEncoder2.getCurrentPosition() / 360 * 1.173150521;
        clearLeft = -leftEncoder2.getCurrentPosition() / 360 * 1.178221633;
        clearBack = frontEncoder2.getCurrentPosition() / 360 * 1.17584979;
    }

    public void Encoders() {
            /*telemetry.addData("True back", backEncoder / 360 * 1.17584979);
            telemetry.addData("True right", rightEncoder / 360 * 1.173150521);
            telemetry.addData("True left", leftEncoder / 360 * 1.178221633);
            telemetry.addData("Right Encoder CM", rightEncoder);
            telemetry.addData("Left Encoder CM", leftEncoder);
            telemetry.addData("Back Encoder CM", backEncoder);
            telemetry.addData("Right Encoder", back_right_wheel.getCurrentPosition());
            telemetry.addData("Left Encoder", front_right_wheel.getCurrentPosition());
            telemetry.addData("Back Encoder", front_left_wheel.getCurrentPosition());
            telemetry.addData("Drive", trueDrive);
            telemetry.addData("Strafe", trueStrafe);
            telemetry.addData("Rotate", trueRotate);
            telemetry.update();
            */
    }

    public void SetIMUMotors(double drive, double strafe, double rotate, double IMU_zAngle) {
        IMUDrive = strafe * Math.sin(-IMU_zAngle) + drive * Math.sin((Math.PI / 2) - IMU_zAngle);
        IMUStrafe = strafe * Math.cos(-IMU_zAngle) + drive * Math.cos((Math.PI / 2) - IMU_zAngle);
        this.SetMotors(IMUDrive, IMUStrafe, rotate);
    }

    public void SetMotors(double drive, double strafe, double rotate) {
        //this.frontLeft = drive - strafe - rotate;
        //this.backLeft = -drive - strafe + rotate;
        //this.frontRight = drive + strafe + rotate;
        //this.backRight = drive - strafe + rotate;
        this.frontLeft = drive + strafe - rotate;
        this.backLeft = -drive + strafe + rotate;
        this.frontRight = drive - strafe + rotate;
        this.backRight = drive + strafe + rotate;
    }


    public void Drive() {
        rightFront.setPower(this.frontRight);
        leftFront.setPower(this.frontLeft);
        leftRear.setPower(this.backLeft);
        rightRear.setPower(-this.backRight);
    }

    public void testDrive(){
        rightFront.setPower(0.3);
        leftFront.setPower(0.3);
        leftRear.setPower(0.3);
        rightRear.setPower(0.3);
    }
}