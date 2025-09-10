package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Autonomous(name="vic's investigative journalist haven", group="Autonomous")
public class VicAuto extends LinearOpMode {
    static final double countsPerMotorRev = 537.7; //gobilda PPR 537.7
    static final double wheelCircumferenceInches = (104 / 25.4) * Math.PI;
    static final double countsPerInch = countsPerMotorRev / wheelCircumferenceInches;
    static final double botDiameter = 25; //24.134;
    static final double inchesPerDegree = (botDiameter * Math.PI) / 360;
    static final double countsPerDegree = countsPerInch * inchesPerDegree;
    static final double countsPerDeadRev = 8192;
    static final double countsPerDeadInch = countsPerDeadRev / ((60 / 25.4) * Math.PI);// 853
    static BasicHardware elsie = new BasicHardware();


    @Override
    public void runOpMode() {
        elsie.initialize(this.hardwareMap);
        elsie.imu.resetYaw();

        //TURNING: POSITIVE = COUNTERCLOCKWISE ROTATION, NEGATIVE = CLOCKWISE ROTATION

        resetMotors();
        waitForStart();

        dumbDrive(36,4);

    }

    private double drive(double speed, double inches, double threshold, double heading) {
        if(opModeIsActive()) {
            int initialValue = elsie.driveOd.getCurrentPosition(); //rightSlide is also the slot the dead wheel encoder is in.

            dumbDrive(inches, 4);

            double movedInches = (initialValue - elsie.driveOd.getCurrentPosition()) / countsPerDeadInch;

            telemetry.addData("moved", movedInches);
            telemetry.addData("error", inches - movedInches);
            telemetry.update();


            if (Math.abs(inches - movedInches) < threshold) {
                point(speed, heading, 1);
                telemetry.addData("done", (Math.abs(inches - movedInches) < threshold));
                telemetry.update();
                return inches - movedInches;
            }
            return drive(speed, inches - movedInches, threshold, heading);
        }
        return 0;
    }
    private void dumbDrive(double inches, double h) {
        resetMotors();
        double speed;

        elsie.backLeft.setTargetPosition((int) Math.round(inches * countsPerInch));
        elsie.backRight.setTargetPosition((int) Math.round(inches * countsPerInch));
        elsie.frontLeft.setTargetPosition((int) Math.round(inches * countsPerInch));
        elsie.frontRight.setTargetPosition((int) Math.round(inches * countsPerInch));

        elsie.backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elsie.backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elsie.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elsie.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        int startPosition = elsie.strafeOd.getCurrentPosition();
        double currentPosition;
        //Start
        while (busy()) {
            currentPosition = -(elsie.strafeOd.getCurrentPosition() - startPosition) / countsPerDeadInch;
            if (currentPosition <= (inches - h)) {
                speed = 1;
            } else {
                speed = (inches - currentPosition) / h;
            }
            telemetry.addData("moved inches", (elsie.strafeOd.getCurrentPosition() - startPosition) / countsPerDeadInch);
            telemetry.addData("speed", speed);
            telemetry.update();
            elsie.backLeft.setPower(speed);
            elsie.backRight.setPower(speed);
            elsie.frontLeft.setPower(speed);
            elsie.frontRight.setPower(speed);
        }

        resetMotors();
    }
    private double strafe(double speed, double inches, double threshold, double heading) {
        if(opModeIsActive()){
            int initialValue = elsie.strafeOd.getCurrentPosition(); //rightSlide is also the slot the dead wheel encoder is in.

            if (speed != 1) {
                stupidStrafe(speed, inches, 0);
            } else {
                stupidStrafe(speed, inches * 0.85 , 0);
            }

            double movedInches = (initialValue - elsie.strafeOd.getCurrentPosition()) / countsPerDeadInch;

            telemetry.addData("moved", movedInches);
            telemetry.addData("error", inches - movedInches);
            telemetry.addData("done", (Math.abs(inches - movedInches) < threshold));
            telemetry.update();

            if (Math.abs(inches - movedInches) < threshold) {
                point(speed,heading,0.65);
                return inches - movedInches;
            }

            strafe(speed, inches - movedInches, threshold, heading);
        }
        return 0;
    }
    private void stupidStrafe(double speed, double inches, int msToWaitAfter) {
        resetMotors();

        elsie.backLeft.setTargetPosition(-((int) Math.round(inches * countsPerInch)));
        elsie.backRight.setTargetPosition((int) Math.round(inches * countsPerInch));
        elsie.frontLeft.setTargetPosition((int) Math.round(inches * countsPerInch));
        elsie.frontRight.setTargetPosition(-((int) Math.round(inches * countsPerInch)));

        elsie.backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elsie.backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elsie.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elsie.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //Start
        elsie.backLeft.setPower(-speed);
        elsie.backRight.setPower(speed);
        elsie.frontLeft.setPower(speed);
        elsie.frontRight.setPower(-speed);

        while (busy()) {
            telemetry.addLine("Running..");
            telemetry.update();
        }

        resetMotors();
        sleep(msToWaitAfter);
    }
    private double point(double power, double direction, double tolerance) {
        //start turn algorithm; generates the amount of turning necessary by subtracting ideal heading from current heading
        return turn(power, direction - elsie.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES),tolerance);
    }
    private double turn(double power, double turnDegrees, double tolerance) {
        //stores current heading
        double botHeading = elsie.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        telemetry.addData("direction", botHeading);
        telemetry.addData("error",turnDegrees);

        turnDegrees = ((turnDegrees % 360) + 540) % 360 - 180; //https://www.desmos.com/calculator/p3xcs1v72p

        telemetry.update();

        if (Math.abs(turnDegrees) <= tolerance) {
            //if the amount the bot is trying to turn is within tolerances, end the algorithm
            telemetry.update();
            return turnDegrees;
        }
        attemptTurn(power, turnDegrees); //attempt to turn the amount necessary

        telemetry.addData("final direction", elsie.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
        //store amount of degrees bot has actually moved by subtracting
        // the current heading from the previous heading
        double movedDegrees = (elsie.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) - botHeading);
        //start the loop over again after subtracting the amount of turning necessary
        // by the amount of turning that has actually occurred
        return turn(power * 0.8, turnDegrees - movedDegrees, tolerance);
    }
    private void attemptTurn(double speed, double turnInches) {
        resetMotors();
        elsie.backLeft.setTargetPosition(-(int) Math.round(turnInches * countsPerDegree));
        elsie.backRight.setTargetPosition(((int) Math.round(turnInches * countsPerDegree)));
        elsie.frontLeft.setTargetPosition(-(int) Math.round(turnInches * countsPerDegree));
        elsie.frontRight.setTargetPosition(((int) Math.round(turnInches * countsPerDegree)));

        elsie.backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elsie.backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elsie.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elsie.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //Start
        elsie.backLeft.setPower(-speed);
        elsie.backRight.setPower(speed);
        elsie.frontLeft.setPower(-speed);
        elsie.frontRight.setPower(speed);

        while (busy()) {}

        resetMotors();
    }
    private void cheapSpline(double x, double y, double speed) {
        int sY, sX, dY, dX;
        double botHeading,rx,ry,fL,bL,fR,bR,rdX,rdY,cX = 0,cY = 0;

        sY = elsie.driveOd.getCurrentPosition();
        sX = elsie.strafeOd.getCurrentPosition();
        botHeading = elsie.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        resetMotors();

        elsie.backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elsie.backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elsie.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elsie.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        rx = x * Math.cos(-botHeading) - y * Math.sin(-botHeading); //rx: rotated x
        ry = x * Math.sin(-botHeading) + y * Math.cos(-botHeading); //ry: rotated y
        fL = (ry + rx);
        bL = (ry - rx);
        fR = (ry - rx);
        bR = (ry + rx);
        elsie.frontLeft.setTargetPosition((int) Math.round(fL * countsPerInch));
        elsie.backLeft.setTargetPosition((int) Math.round(bL * countsPerInch));
        elsie.frontRight.setTargetPosition((int) Math.round(fR * countsPerInch));
        elsie.backRight.setTargetPosition((int) Math.round(bR * countsPerInch));

        elsie.frontLeft.setPower(fL / Math.max(Math.abs(fL), 1));
        elsie.backLeft.setPower(bL / Math.max(Math.abs(fL), 1));
        elsie.frontRight.setPower(fR / Math.max(Math.abs(fL), 1));
        elsie.backRight.setPower(bR / Math.max(Math.abs(fL), 1));

        while (busy()) {
            dY = elsie.driveOd.getCurrentPosition() - sY;
            dX = elsie.strafeOd.getCurrentPosition() - sX;
            sY = elsie.driveOd.getCurrentPosition();
            sX = elsie.strafeOd.getCurrentPosition();

            botHeading = elsie.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            rdX = dX * Math.sin(botHeading) + dY * Math.cos(botHeading);
            rdY = dX * Math.cos(botHeading) + dY * Math.sin(botHeading);
            cX += rdX;
            cY += rdY;

            rx = x * Math.cos(-botHeading) - y * Math.sin(-botHeading); //rx: rotated x
            ry = x * Math.sin(-botHeading) + y * Math.cos(-botHeading); //ry: rotated y
            fL = (ry + rx);
            bL = (ry - rx);
            fR = (ry - rx);
            bR = (ry + rx);

            elsie.frontLeft.setTargetPosition((int) Math.round(fL * countsPerInch));
            elsie.backLeft.setTargetPosition((int) Math.round(bL * countsPerInch));
            elsie.frontRight.setTargetPosition((int) Math.round(fR * countsPerInch));
            elsie.backRight.setTargetPosition((int) Math.round(bR * countsPerInch));

            elsie.frontLeft.setPower(speed * fL / Math.abs(fL));
            elsie.backLeft.setPower(speed * bL / Math.abs(fL));
            elsie.frontRight.setPower(speed * fR / Math.abs(fL));
            elsie.backRight.setPower(speed * bR / Math.abs(fL));

            telemetry.addData("bot heading", Math.toDegrees(botHeading));
            //telemetry.addData("x", x);
            //telemetry.addData("y", y);
            telemetry.addData("rx", rx);
            telemetry.addData("ry", ry);
            telemetry.addData("dX", dX);// / countsPerDeadInch);
            telemetry.addData("dY", dY);// / countsPerDeadInch);
            telemetry.update();
        }
        resetMotors();
    }
    private void spline(double y, double x, double turn) {
        int sY, sX, dY, dX;
        double botHeading,rx,ry,fL,bL,fR,bR,rdX,rdY,cX = 0,cY = 0;

        sY = elsie.driveOd.getCurrentPosition();
        sX = elsie.strafeOd.getCurrentPosition();
        botHeading = elsie.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        turn *= inchesPerDegree; //convert turn to a wheel movement

        resetMotors();

        elsie.backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elsie.backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elsie.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elsie.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        rx = x * Math.cos(-botHeading) - y * Math.sin(-botHeading); //rx: rotated x
        ry = x * Math.sin(-botHeading) + y * Math.cos(-botHeading); //ry: rotated y
        fL = (ry + rx + turn);
        bL = (ry - rx + turn);
        fR = (ry - rx - turn);
        bR = (ry + rx - turn);
        elsie.frontLeft.setTargetPosition((int) Math.round(fL * countsPerInch));
        elsie.backLeft.setTargetPosition((int) Math.round(bL * countsPerInch));
        elsie.frontRight.setTargetPosition((int) Math.round(fR * countsPerInch));
        elsie.backRight.setTargetPosition((int) Math.round(bR * countsPerInch));
        elsie.frontLeft.setPower(fL / Math.max(Math.abs(fL), 1));
        elsie.backLeft.setPower(bL / Math.max(Math.abs(fL), 1));
        elsie.frontRight.setPower(fR / Math.max(Math.abs(fL), 1));
        elsie.backRight.setPower(bR / Math.max(Math.abs(fL), 1));

        while (busy()) {
            dY = elsie.driveOd.getCurrentPosition() - sY;
            dX = elsie.strafeOd.getCurrentPosition() - sX;
            sY = elsie.driveOd.getCurrentPosition();
            sX = elsie.strafeOd.getCurrentPosition();

            botHeading = elsie.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            rdX = dX * Math.cos(botHeading) - dY * Math.sin(botHeading);
            rdY = dY * Math.sin(botHeading) + dY * Math.cos(botHeading);
            cX += rdX;
            cY += rdY;

            rx = x * Math.cos(-botHeading) - y * Math.sin(-botHeading); //rx: rotated x
            ry = x * Math.sin(-botHeading) + y * Math.cos(-botHeading); //ry: rotated y
            fL = (ry + rx + turn);
            bL = (ry - rx + turn);
            fR = (ry - rx - turn);
            bR = (ry + rx - turn);

            elsie.frontLeft.setTargetPosition((int) Math.round(fL * countsPerInch));
            elsie.backLeft.setTargetPosition((int) Math.round(bL * countsPerInch));
            elsie.frontRight.setTargetPosition((int) Math.round(fR * countsPerInch));
            elsie.backRight.setTargetPosition((int) Math.round(bR * countsPerInch));

            elsie.frontLeft.setPower(fL / Math.abs(fL));
            elsie.backLeft.setPower(bL / Math.abs(fL));
            elsie.frontRight.setPower(fR / Math.abs(fL));
            elsie.backRight.setPower(bR / Math.abs(fL));

            telemetry.addData("bot heading", Math.toDegrees(botHeading));
            //telemetry.addData("x", x);
            //telemetry.addData("y", y);
            telemetry.addData("rx", rx);
            telemetry.addData("ry", ry);
            telemetry.addData("cX", cX / countsPerDeadInch);
            telemetry.addData("cY", cY  / countsPerDeadInch);
            telemetry.update();
        }
        resetMotors();
        while(opModeIsActive()){
            telemetry.addData("bot heading", Math.toDegrees(botHeading));
            telemetry.addData("rx", rx);
            telemetry.addData("ry", ry);
            telemetry.addData("cX", cX / countsPerDeadInch);
            telemetry.addData("cY", cY  / countsPerDeadInch);
            telemetry.update();
        }
    }
    private void resetMotors() {
        elsie.backLeft.setPower(0);
        elsie.backRight.setPower(0);
        elsie.frontLeft.setPower(0);
        elsie.frontRight.setPower(0);

        elsie.backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        elsie.backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        elsie.frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        elsie.frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        elsie.backLeft.setTargetPosition(elsie.backLeft.getCurrentPosition());
        elsie.backRight.setTargetPosition(elsie.backRight.getCurrentPosition());
        elsie.frontLeft.setTargetPosition(elsie.frontLeft.getCurrentPosition());
        elsie.frontRight.setTargetPosition(elsie.frontLeft.getCurrentPosition());

        elsie.backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elsie.backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elsie.frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elsie.frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elsie.backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        elsie.backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        elsie.frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        elsie.frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    private boolean busy() {
        return opModeIsActive() && (elsie.backLeft.isBusy() || elsie.backRight.isBusy() || elsie.frontLeft.isBusy() || elsie.frontRight.isBusy());
    }
}
