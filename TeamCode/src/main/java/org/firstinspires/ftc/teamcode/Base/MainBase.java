package org.firstinspires.ftc.teamcode.Base;

import android.os.SystemClock;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class MainBase {

    // Total Motors: 5
    // Total Servos: 3
    public DcMotor leftDT      = null;
    public DcMotor rightDT     = null;
    public DcMotor leftDuck    = null;
    public DcMotor rightDuck   = null;
    public DcMotor lift        = null;
    public CRServo rightClaw   = null;
    public CRServo bucket      = null;
    public Servo   leftClaw    = null;


    // Total Sensors: 2
    /*public ModernRoboticsI2cRangeSensor frontRange   = null;
    public ModernRoboticsI2cRangeSensor leftRange    = null;
    public ModernRoboticsI2cRangeSensor rightRange   = null;
    public ModernRoboticsI2cGyro        gyro         = null;*/


    static final double     COUNTS_PER_MOTOR_REV    = 386.3;
    static final double     DRIVE_GEAR_REDUCTION    = 1.0;
    static final double     WHEEL_DIAMETER_INCHES   = 2.5;
    public static final double     COUNTS_PER_INCH  = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION)
            / (WHEEL_DIAMETER_INCHES * 3.14159265);
    public double                  amountError        = 1.0;
    public static final double     DRIVE_SPEED        = 1.0;
    public static final double     TURN_SPEED         = 0.5;
    public static final double     P_TURN_COEFF       = 0.1;
    public static final double     HEADING_THRESHOLD  = 1.0;
    public static final double   MAX_ACCEPTABLE_ERROR = 10;
    public double                                 rpm = 0;

    HardwareMap hwMap = null;

    public void init(HardwareMap ahwMap) {
        hwMap = ahwMap;

        leftDT    = hwMap.get(DcMotor.class, "leftDT");
        rightDT   = hwMap.get(DcMotor.class, "rightDT");
        leftDuck  = hwMap.get(DcMotor.class, "leftDuck");
        rightDuck = hwMap.get(DcMotor.class, "rightDuck");
        lift      = hwMap.get(DcMotor.class, "lift");
        rightClaw = hwMap.get(CRServo.class,"rightClaw");
        bucket    = hwMap.get(CRServo.class,"bucket");
        leftClaw  = hwMap.get(Servo.class, "leftClaw");

        leftDT.setDirection(DcMotor.Direction.REVERSE);
        rightDT.setDirection(DcMotor.Direction.FORWARD);

        //leftDT.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        //rightDT.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        leftDT.setPower(0);
        rightDT.setPower(0);
        leftDuck.setPower(0);
        rightDuck.setPower(0);
        lift.setPower(0);
        rightClaw.setPower(0);
        bucket.setPower(0);
        leftClaw.setPosition(0);


        /*frontRange = hwMap.get(ModernRoboticsI2cRangeSensor.class,"frontRange");
        frontRange.initialize();

        leftRange  = hwMap.get(ModernRoboticsI2cRangeSensor.class,"leftRange");
        leftRange.initialize();

        rightRange  = hwMap.get(ModernRoboticsI2cRangeSensor.class,"rightRange");
        rightRange.initialize();

        gyro = hwMap.get(ModernRoboticsI2cGyro.class,"gyro");
        gyro.initialize();
        gyro.calibrate();*/

    }

    /*public boolean getCurrentRPM(double initTime, double currentTime, int initPos, int currentPos, LinearOpMode opMode){
        double differenceInTime = currentTime - initTime;
        if(differenceInTime > 1){
            int differenceInPos = currentPos - initPos;
            double revolutions = differenceInPos / 28;
            double minutes = differenceInTime / 60;
            rpm = revolutions / minutes;
        }
        opMode.telemetry.addLine("Current RPM: " + rpm);
        if(differenceInTime > 1) return true;
        return false;
    }

    /*public void getToTargetSpeed(int target_rpm){
        double percentOfTotal = (double)(target_rpm)/((double)5100);
        rightDuck.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDuck.setPower(percentOfTotal);
    }*/

    /*public double getError (double angle){

        double robotError;

        // Calculates error from angle.
        robotError = angle - gyro.getIntegratedZValue();
        while (robotError > 180)  robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    public double getSteer (double error, double P_DRIVE_COEFF){
        return Range.clip(error * P_DRIVE_COEFF, -1, 1);
    }

    //Autonomous driving method that utilizes gyroscope to correct angle veer-offs during strafing
    public void gyroDrive (double speed, double distanceTL, double distanceTR,
                           double distanceBL, double distanceBR, double angle, double endFLPower, double endFRPower, double endBLPower,
                           double endBRPower, LinearOpMode opmode) {

        int     newTLTarget;
        int     newTRTarget;
        int     newBLTarget;
        int     newBRTarget;
        int     moveCountsTL = (int)(distanceTL * MainBase.COUNTS_PER_INCH);
        int     moveCountsTR = (int)(distanceTR * MainBase.COUNTS_PER_INCH);
        int     moveCountsBL = (int)(distanceBL * MainBase.COUNTS_PER_INCH);
        int     moveCountsBR = (int)(distanceBR * MainBase.COUNTS_PER_INCH);
        double  max;
        double  leftMax;
        double  rightMax;
        double  error;
        double  speedTL;
        double  speedTR;
        double  speedBL;
        double  speedBR;
        double  ErrorAmount;
        boolean goodEnough = false;

        // Ensure that the Op-mode is still active
        if (opmode.opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newTLTarget  = leftDT.getCurrentPosition() + moveCountsTL;
            newTRTarget  = rightDT.getCurrentPosition() + moveCountsTR;
            newBLTarget  = backLeft.getCurrentPosition() + moveCountsBL;
            newBRTarget  = backRight.getCurrentPosition() + moveCountsBR;

            // Set Target and Turn On RUN_TO_POSITION
            leftDT.setTargetPosition(newTLTarget);
            rightDT.setTargetPosition(newTRTarget);
            backLeft.setTargetPosition(newBLTarget);
            backRight.setTargetPosition(newBRTarget);

            leftDT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightDT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // start motion
            speed = Range.clip(Math.abs(speed), 0.0, 1.0);
            leftDT.setPower(speed);
            rightDT.setPower(speed);
            backLeft.setPower(speed);
            backRight.setPower(speed);

            // keep looping while we are still active, and ALL motors are running.
            while (opmode.opModeIsActive() &&
                    (leftDT.isBusy() || rightDT.isBusy() || backLeft.isBusy() || backRight.isBusy()) && !goodEnough) {

                // adjust relative speed based on heading error.
                error = getError(angle);
                double steer = getSteer(error, 0.15);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distanceTL > 0)
                    speedTL  = speed - steer;
                else speedTL = speed + steer;

                if (distanceTR > 0)
                    speedTR  = speed + steer;
                else speedTR = speed - steer;

                if (distanceBL > 0)
                    speedBL  = speed - steer;
                else speedBL = speed + steer;

                if (distanceBR > 0)
                    speedBR  = speed + steer;
                else speedBR = speed - steer;

                // Normalize speeds if either one exceeds +/- 1.0
                leftMax  = Math.max(Math.abs(speedTL), Math.abs(speedTR));
                rightMax = Math.max(Math.abs(speedBL), Math.abs(speedBR));
                max      = Math.max(leftMax,rightMax);
                if (max > 1.0)
                {
                    speedTL /= max;
                    speedTR /= max;
                    speedBL /= max;
                    speedBR /= max;
                }

                ErrorAmount = ((Math.abs(((newBLTarget) - (backLeft.getCurrentPosition())))
                        + (Math.abs(((newTLTarget) - (leftDT.getCurrentPosition()))))
                        + (Math.abs((newBRTarget) - (backRight.getCurrentPosition())))
                        + (Math.abs(((newTRTarget) - (rightDT.getCurrentPosition()))))) / COUNTS_PER_INCH);
                if (ErrorAmount < amountError) {
                    goodEnough = true;
                }

                leftDT.setPower(speedTL);
                rightDT.setPower(speedTR);
                backLeft.setPower(speedBL);
                backRight.setPower(speedBR);

                // Display drive status for the driver.
                /*opmode.telemetry.addData("Err/St",  "%5.1f/%5.1f",  error, steer);
                opmode.telemetry.addData("Target",  "%7d:%7d:%7d:%7d", newTLTarget, newTRTarget, newBLTarget, newBRTarget);
                opmode.telemetry.addData("Actual",  "%7d:%7d:%7d:%7d", leftDT.getCurrentPosition(),
                        rightDT.getCurrentPosition(), backLeft.getCurrentPosition(), backRight.getCurrentPosition() );
                opmode.telemetry.addData("Speed",   "%5.2f:%5.2f:%5.2f:%5.2f", speedTL, speedTR, speedBL, speedBR);*/

                /*opmode.telemetry.addData("FL: ", leftDT.isBusy());
                opmode.telemetry.addData("FR: ", rightDT.isBusy());
                opmode.telemetry.addData("BL: ", backLeft.isBusy());
                opmode.telemetry.addData("BR: ", backRight.isBusy());
                opmode.telemetry.addData("Good Enough: ", goodEnough);
                opmode.telemetry.addData("Error Amount: ", ErrorAmount);
                opmode.telemetry.addData("Amount Error: ", amountError);
                opmode.telemetry.update();
            }

            leftDT.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightDT.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            leftDT.setPower(endFLPower);
            rightDT.setPower(endFRPower);
            backLeft.setPower(endBLPower);
            backRight.setPower(endBRPower);
        }
    }

    public void encoderDrive ( double speed,
                               double leftDTInches, double rightDTInches, double backLeftInches,
                               double backRightInches, LinearOpMode opMode){
        int newleftDTTarget;
        int newrightDTTarget;
        int newbackLeftTarget;
        int newbackRightTarget;
        double ErrorAmount;
        boolean goodEnough = false;

        // Ensure that the opmode is still active
        if (opMode.opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newleftDTTarget = leftDT.getCurrentPosition() + (int) (leftDTInches * COUNTS_PER_INCH);
            newrightDTTarget = rightDT.getCurrentPosition() + (int) (rightDTInches * COUNTS_PER_INCH);
            newbackLeftTarget = backLeft.getCurrentPosition() + (int) (backLeftInches * COUNTS_PER_INCH);
            newbackRightTarget = backRight.getCurrentPosition() + (int) (backRightInches * COUNTS_PER_INCH);
            leftDT.setTargetPosition(newleftDTTarget);
            rightDT.setTargetPosition(newrightDTTarget);
            backLeft.setTargetPosition(newbackLeftTarget);
            backRight.setTargetPosition(newbackRightTarget);

            // Turn On RUN_TO_POSITION
            leftDT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightDT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            leftDT.setPower(Math.abs(speed));
            rightDT.setPower(Math.abs(speed));
            backLeft.setPower(Math.abs(speed));
            backRight.setPower(Math.abs(speed));

            while (opMode.opModeIsActive() &&
                    ((leftDT.isBusy() || rightDT.isBusy()) || (backLeft.isBusy() || backRight.isBusy())) && !goodEnough) {

                // Display it for the driver.
                opMode.telemetry.addData("Path1", "Running to %7d :%7d", newleftDTTarget, newbackLeftTarget, newrightDTTarget, newbackRightTarget);
                opMode.telemetry.addData("Path2", "Running at %7d :%7d",

                        leftDT.getCurrentPosition(),
                        rightDT.getCurrentPosition(),
                        backLeft.getCurrentPosition(),
                        backRight.getCurrentPosition());
                opMode.telemetry.addData("leftDT", leftDT.getCurrentPosition());
                opMode.telemetry.addData("backLeft", backLeft.getCurrentPosition());
                opMode.telemetry.addData("rightDT", rightDT.getCurrentPosition());
                opMode.telemetry.addData("backRight", backRight.getCurrentPosition());

                opMode.telemetry.update();

                ErrorAmount = ((Math.abs(((newbackLeftTarget) - (backLeft.getCurrentPosition())))
                        + (Math.abs(((newleftDTTarget) - (leftDT.getCurrentPosition()))))
                        + (Math.abs((newbackRightTarget) - (backRight.getCurrentPosition())))
                        + (Math.abs(((newrightDTTarget) - (rightDT.getCurrentPosition()))))) / COUNTS_PER_INCH);
                if (ErrorAmount < amountError) {
                    goodEnough = true;
                }
            }

            leftDT.setPower(0);
            rightDT.setPower(0);
            backLeft.setPower(0);
            backRight.setPower(0);

            // Turn off RUN_TO_POSITION
            leftDT.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightDT.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    //3-Check Function for rangeDrive
    private boolean rangeCheck(ModernRoboticsI2cRangeSensor range_sensor, double desired_distance, LinearOpMode opMode){
        final int TRIES = 3;
        for (int i = 0; i < TRIES; i++){
            if (Math.abs(range_sensor.getDistance(DistanceUnit.INCH) - desired_distance) < MAX_ACCEPTABLE_ERROR){
                return true;
            }
            opMode.telemetry.addData("TRY ",i);
            opMode.telemetry.addData("Range Value: ", range_sensor.getDistance(DistanceUnit.INCH));
            opMode.telemetry.addData("Target: ", desired_distance);
            opMode.telemetry.update();
            try {
                Thread.sleep(1500);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
        return false;
    }

    //Utilization of Range Sensors in Autonomous
    public void rangeDrive (double speed, double frontDistance, double leftDistance, double rightDistance, LinearOpMode opmode) {

        speed = Range.clip(Math.abs(speed), 0.0, 1.0);

        if(frontDistance != -1) {
            while (frontRange.getDistance(DistanceUnit.INCH) < frontDistance){
                if (Math.abs(frontRange.getDistance(DistanceUnit.INCH) - frontDistance) > MAX_ACCEPTABLE_ERROR){
                    if (!rangeCheck(frontRange,frontDistance, opmode)){
                        break;
                    }
                }
                leftDT.setPower(-speed);
                rightDT.setPower(-speed);
                backLeft.setPower(-speed);
                backRight.setPower(-speed);

                opmode.telemetry.addData("Sensor Front Distance: ", frontRange.getDistance(DistanceUnit.INCH));
                opmode.telemetry.addData("Target Front Distance: ", frontDistance);
                opmode.telemetry.addLine("Moving Backwards");
                opmode.telemetry.update();
            }
            while (frontRange.getDistance(DistanceUnit.INCH) > frontDistance){
                if (Math.abs(frontRange.getDistance(DistanceUnit.INCH) - frontDistance) > MAX_ACCEPTABLE_ERROR){
                    if (!rangeCheck(frontRange,frontDistance,opmode)){
                        break;
                    }
                }
                leftDT.setPower(speed);
                rightDT.setPower(speed);
                backLeft.setPower(speed);
                backRight.setPower(speed);

                opmode.telemetry.addData("Sensor Front Distance: ", frontRange.getDistance(DistanceUnit.INCH));
                opmode.telemetry.addData("Target Front Distance: ", frontDistance);
                opmode.telemetry.addLine("Moving Forwards");
                opmode.telemetry.update();
            }
        }
        if (leftDistance != -1) {
            while (leftRange.getDistance(DistanceUnit.INCH) < leftDistance){
                if (Math.abs(leftRange.getDistance(DistanceUnit.INCH) - leftDistance) > MAX_ACCEPTABLE_ERROR){
                    if (!rangeCheck(leftRange,leftDistance,opmode)){
                        break;
                    }
                }
                leftDT.setPower(speed);
                rightDT.setPower(-speed);
                backLeft.setPower(-speed);
                backRight.setPower(speed);

                opmode.telemetry.addData("Sensor Left Distance: ", frontRange.getDistance(DistanceUnit.INCH));
                opmode.telemetry.addData("Target Left Distance: ", frontDistance);
                opmode.telemetry.addLine("Moving Right");
                opmode.telemetry.update();
            }
            while (leftRange.getDistance(DistanceUnit.INCH) > leftDistance){
                if (Math.abs(leftRange.getDistance(DistanceUnit.INCH) - leftDistance) > MAX_ACCEPTABLE_ERROR){
                    if (!rangeCheck(leftRange,leftDistance,opmode)){
                        break;
                    }
                }
                leftDT.setPower(-speed);
                rightDT.setPower(speed);
                backLeft.setPower(speed);
                backRight.setPower(-speed);

                opmode.telemetry.addData("Sensor Left Distance: ", frontRange.getDistance(DistanceUnit.INCH));
                opmode.telemetry.addData("Target Left Distance: ", frontDistance);
                opmode.telemetry.addLine("Moving Left");
                opmode.telemetry.update();
            }
        }
        if (rightDistance != -1){
            while (rightRange.getDistance(DistanceUnit.INCH) > rightDistance){
                if (Math.abs(rightRange.getDistance(DistanceUnit.INCH) - rightDistance) > MAX_ACCEPTABLE_ERROR){
                    if (!rangeCheck(rightRange,rightDistance,opmode)){
                        break;
                    }
                }
                leftDT.setPower(speed);
                rightDT.setPower(-speed);
                backLeft.setPower(-speed);
                backRight.setPower(speed);

                opmode.telemetry.addData("Sensor Right Distance: ", frontRange.getDistance(DistanceUnit.INCH));
                opmode.telemetry.addData("Target Right Distance: ", frontDistance);
                opmode.telemetry.addLine("Moving Right");
                opmode.telemetry.update();
            }
            while (rightRange.getDistance(DistanceUnit.INCH) < rightDistance){
                if (Math.abs(rightRange.getDistance(DistanceUnit.INCH) - rightDistance) > MAX_ACCEPTABLE_ERROR){
                    if (!rangeCheck(rightRange,rightDistance,opmode)){
                        break;
                    }
                }
                leftDT.setPower(-speed);
                rightDT.setPower(speed);
                backLeft.setPower(speed);
                backRight.setPower(-speed);

                opmode.telemetry.addData("Sensor Right Distance: ", frontRange.getDistance(DistanceUnit.INCH));
                opmode.telemetry.addData("Target Right Distance: ", frontDistance);
                opmode.telemetry.addLine("Moving Left");
                opmode.telemetry.update();
            }
        }
    }

    public void gyroTurn (double speed, double angle, LinearOpMode opmode) {

        // keep looping while we are still active, and not on heading.
        while (opmode.opModeIsActive() && !onHeading(speed, angle, P_TURN_COEFF, opmode)) {
            // Update telemetry & Allow time for other processes to run.
            opmode.telemetry.update();
        }
    }
    public void sleepV2(double wait, LinearOpMode opmode){
        double finalTime = opmode.time + wait;
        while(finalTime > opmode.time){
            opmode.telemetry.update();
        }
    }

    boolean onHeading(double speed, double angle, double PCoeff, LinearOpMode opmode) {
        double   error ;
        double   steer ;
        boolean  onTarget = false ;
        double leftSpeed;
        double rightSpeed;

        // determine turn power based on +/- error
        error = getError(angle);

        if (Math.abs(error) <= HEADING_THRESHOLD) {
            steer = 0.0;
            leftSpeed  = 0.0;
            rightSpeed = 0.0;
            onTarget = true;
        }
        else {
            steer = getSteer(error, PCoeff);
            rightSpeed  = speed * steer;
            leftSpeed   = -rightSpeed;
        }

        // Send desired speeds to motors.
        leftDT.setPower(leftSpeed);
        rightDT.setPower(rightSpeed);
        backLeft.setPower(leftSpeed);
        backRight.setPower(rightSpeed);

        // Display it for the driver.
        opmode.telemetry.addData("Target", "%5.2f", angle);
        opmode.telemetry.addData("Err/St", "%5.2f/%5.2f", error, steer);
        opmode.telemetry.addData("Speed ", "%5.2f:%5.2f", leftSpeed, rightSpeed);

        return onTarget;
    }
    */
}