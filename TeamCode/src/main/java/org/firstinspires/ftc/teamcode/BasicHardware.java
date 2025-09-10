
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
public class BasicHardware {
    protected DcMotor backLeft, frontLeft, backRight, frontRight, driveOd, strafeOd, cannon, cannon2, leftSlide;
    protected Servo leftClaw, rightClaw, leftArm, rightArm;
    protected ColorSensor colorSensor;
    protected static final boolean USE_WEBCAM = true;
    protected IMU imu;
    protected void initialize(HardwareMap hMap) {


        backLeft  = hMap.get(DcMotor.class, "backLeft");
        backRight  = hMap.get(DcMotor.class, "backRight");
        frontLeft = hMap.get(DcMotor.class, "frontLeft");
        frontRight = hMap.get(DcMotor.class, "frontRight");

        cannon = hMap.get(DcMotor.class, "cannon");
        cannon2 = hMap.get(DcMotor.class, "cannon2");

        driveOd = hMap.get(DcMotor.class, "driveOd");
        strafeOd = hMap.get(DcMotor.class, "strafeOd");

        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.FORWARD);

        cannon.setDirection(DcMotor.Direction.FORWARD);
        cannon2.setDirection(DcMotorSimple.Direction.REVERSE);

        driveOd.setDirection(DcMotor.Direction.REVERSE);
        strafeOd.setDirection(DcMotor.Direction.FORWARD);

        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        cannon.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        cannon2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        driveOd.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        strafeOd.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        imu = hMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.RIGHT
                )
        );
        imu.initialize(parameters);

    }
}
