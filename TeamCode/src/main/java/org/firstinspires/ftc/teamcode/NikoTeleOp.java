package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit; // I NEED ME MY RADIANS BABYYYY

@TeleOp(name="Niko's whimsical land of sobriety", group="TeleOp")
public class NikoTeleOp extends LinearOpMode {
    private BasicHardware elsie = new BasicHardware();
    private double x, y, θ, rx, ry, frontLeftPower, frontRightPower,backLeftPower,backRightPower, botHeading, rightSlidePower, leftSlidePower;

    private double speed = 1;
    @Override
    public void runOpMode() {
        elsie.initialize(this.hardwareMap);

        waitForStart();

        while (opModeIsActive()) {
            elsie.backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            elsie.backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            elsie.frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            elsie.frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            /*elsie.leftSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            elsie.rightSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);*/

            y = -gamepad1.left_stick_y; //y: amount to move on the y axis
            x = gamepad1.left_stick_x; //x: amount to move on the x axis
            θ = gamepad1.right_stick_x;

            if (Math.abs(x) < 0.1)
                x = 0;
            if (Math.abs(y) < 0.1)
                y = 0;
            if (Math.abs(θ) < 0.1)
                θ = 0;

            if (gamepad1.back) {
                elsie.imu.resetYaw();
            }
            if(gamepad1.left_bumper) {
                speed = 0.6;
            } else if(gamepad1.right_bumper) {
                speed = 1;
            }

            botHeading = elsie.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            /*
            if (gamepad1.right_stick_y*gamepad1.right_stick_y+gamepad1.right_stick_x*gamepad1.right_stick_x > 0.1)
                idealBotHeading = Math.atan2(gamepad1.right_stick_x,-gamepad1.right_stick_y);
            θ = (((idealBotHeading - botHeading + 3*Math.PI) % (2*Math.PI)) - Math.PI) % (2 * Math.PI);
            telemetry.addData("absolute θ:", θ);
            if (Math.abs(θ) / Math.PI < 0.1)
                θ = 0;
            θ /= Math.abs(θ) * 10;
            */

            //the bot heading is INVERTED before being plugged into the rotation formula because we want to rotate motion vectors AGAINST the bot's heading
            //i was sober when i wrote this so my math should be ok :))
            rx = x * Math.cos(-botHeading) - y * Math.sin(-botHeading); //rx: rotated x
            ry = x * Math.sin(-botHeading) + y * Math.cos(-botHeading); //ry: rotated y

            double max = Math.max(1, Math.abs(rx) + Math.abs(ry) + Math.abs(θ));

            frontLeftPower = (ry + rx + θ) / max;
            backLeftPower = (ry - rx + θ) / max;
            frontRightPower = (ry - rx - θ) / max;
            backRightPower = (ry + rx - θ) / max;

            // Send calculated power to wheels
            elsie.backLeft.setPower(backLeftPower * speed);
            elsie.frontLeft.setPower(frontLeftPower * speed);
            elsie.backRight.setPower(backRightPower * speed);
            elsie.frontRight.setPower(frontRightPower * speed);

            elsie.cannon.setPower(gamepad2.left_stick_y);
            elsie.cannon2.setPower(gamepad2.left_stick_y);
            
            telemetry.addData("Heading:", Math.toDegrees(botHeading));
            telemetry.addData("θ:", θ);
            telemetry.addData("rx:", rx);
            telemetry.addData("ry:", ry);
            telemetry.addData("left stick", gamepad2.left_stick_y);
            //telemetry.addData("left arm", elsie.leftArm.getPosition());
            telemetry.update();
        }
    }
}