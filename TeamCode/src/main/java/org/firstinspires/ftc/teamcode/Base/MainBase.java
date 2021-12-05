package org.firstinspires.ftc.teamcode.Base;

import android.os.SystemClock;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
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
    public Servo   rightClaw   = null;
    public Servo   bucket      = null;
    public Servo   leftClaw    = null;


    // Total Sensors: 3
    public ModernRoboticsI2cRangeSensor backRange   = null;
    public ModernRoboticsI2cRangeSensor sideRange   = null;
    public ModernRoboticsI2cGyro        gyro        = null;


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
        rightClaw = hwMap.get(Servo.class,"rightClaw");
        bucket    = hwMap.get(Servo.class,"bucket");
        leftClaw  = hwMap.get(Servo.class, "leftClaw");

        backRange = hwMap.get(ModernRoboticsI2cRangeSensor.class,"backRange");
        backRange.initialize();

        sideRange  = hwMap.get(ModernRoboticsI2cRangeSensor.class,"sideRange");
        sideRange.initialize();

        gyro = hwMap.get(ModernRoboticsI2cGyro.class,"gyro");
        gyro.initialize();
        gyro.calibrate();

        leftDT.setDirection(DcMotor.Direction.REVERSE);
        rightDT.setDirection(DcMotor.Direction.REVERSE);

        //leftDT.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        //rightDT.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        leftDT.setPower(0);
        rightDT.setPower(0);
        leftDuck.setPower(0);
        rightDuck.setPower(0);
        lift.setPower(0);
        rightClaw.setPosition(0.3);
        bucket.setPosition(0.3);
        leftClaw.setPosition(0);
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

    public void getToTargetSpeed(int target_rpm){
        double percentOfTotal = (double)(target_rpm)/((double)5100);
        rightDuck.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDuck.setPower(percentOfTotal);
    }*/

    public double getError (double angle){

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
    public void gyroDrive(double speed, double distanceL, double distanceR, double angle,
                          double endFLPower, double endFRPower,
                          LinearOpMode opmode){

        int     newTLTarget;
        int     newTRTarget;
        int     moveCountsTL = (int)(distanceL * MainBase.COUNTS_PER_INCH);
        int     moveCountsTR = (int)(distanceR * MainBase.COUNTS_PER_INCH);

        double  max;
        double  leftMax;
        double  rightMax;
        double  error;
        double  speedL;
        double  speedR;
        double  ErrorAmount;
        boolean goodEnough = false;

        // Ensure that the Op-mode is still active
        if (opmode.opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newTLTarget  = leftDT.getCurrentPosition() + moveCountsTL;
            newTRTarget  = rightDT.getCurrentPosition() + moveCountsTR;

            // Set Target and Turn On RUN_TO_POSITION
            leftDT.setTargetPosition(newTLTarget);
            rightDT.setTargetPosition(newTRTarget);

            leftDT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightDT.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // Start motion
            speed = Range.clip(Math.abs(speed), 0.0, 1.0);
            leftDT.setPower(speed);
            rightDT.setPower(speed);

            // Keep looping while we are still active, and ALL motors are running.
            while (opmode.opModeIsActive() &&
                    (leftDT.isBusy() || rightDT.isBusy()) && !goodEnough) {

                // Adjust relative speed based on heading error.
                error = getError(angle);
                double steer = getSteer(error, 0.15);

                // If driving in reverse, the motor correction also needs to be reversed
                if (distanceL > 0)
                    speedL  = speed - steer;
                else speedL = speed + steer;

                if (distanceR > 0)
                    speedR  = speed + steer;
                else speedR = speed - steer;

                // Normalize speeds if either one exceeds +/- 1.0
                max = Math.max(speedL,speedR);
                if (max > 1.0)
                {
                    speedL /= max;
                    speedR /= max;
                }

                ErrorAmount = (Math.abs(((newTLTarget) - (leftDT.getCurrentPosition()))))
                        + (Math.abs(((newTRTarget) - (rightDT.getCurrentPosition())))) / COUNTS_PER_INCH;
                if (ErrorAmount < amountError) {
                    goodEnough = true;
                }

                leftDT.setPower(speedL);
                rightDT.setPower(speedR);

                // Display drive status for the driver.
                opmode.telemetry.addData("Err/St",  "%5.1f/%5.1f",  error, steer);
                opmode.telemetry.addData("Target",  "%7d:%7d:%7d:%7d", newTLTarget, newTRTarget);
                opmode.telemetry.addData("Actual",  "%7d:%7d:%7d:%7d", leftDT.getCurrentPosition(),
                        rightDT.getCurrentPosition());
                opmode.telemetry.addData("Speed",   "%5.2f:%5.2f:%5.2f:%5.2f", speedL, speedR);

                opmode.telemetry.addData("FL: ", leftDT.isBusy());
                opmode.telemetry.addData("FR: ", rightDT.isBusy());
                opmode.telemetry.addData("Good Enough: ", goodEnough);
                opmode.telemetry.addData("Error Amount: ", ErrorAmount);
                opmode.telemetry.addData("Amount Error: ", amountError);
                opmode.telemetry.update();
            }

            leftDT.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightDT.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            leftDT.setPower(endFLPower);
            rightDT.setPower(endFRPower);
        }
    }

    public void encoderDrive(double speed, double leftDTInches, double rightDTInches,
                             LinearOpMode opMode){
        int newleftDTTarget;
        int newrightDTTarget;
        double ErrorAmount;
        boolean goodEnough = false;

        // Ensure that the opmode is still active
        if (opMode.opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newleftDTTarget = leftDT.getCurrentPosition() + (int) (leftDTInches * COUNTS_PER_INCH);
            newrightDTTarget = rightDT.getCurrentPosition() + (int) (rightDTInches * COUNTS_PER_INCH);
            leftDT.setTargetPosition(newleftDTTarget);
            rightDT.setTargetPosition(newrightDTTarget);

            // Turn On RUN_TO_POSITION
            leftDT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightDT.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            leftDT.setPower(Math.abs(speed));
            rightDT.setPower(Math.abs(speed));

            while (opMode.opModeIsActive() &&
                    ((leftDT.isBusy() || rightDT.isBusy())) && !goodEnough) {

                // Display it for the driver.
                opMode.telemetry.addData("Path1", "Running to %7d :%7d", newleftDTTarget, newrightDTTarget);
                opMode.telemetry.addData("Path2", "Running at %7d :%7d",

                        leftDT.getCurrentPosition(),
                        rightDT.getCurrentPosition(),
                opMode.telemetry.addData("leftDT", leftDT.getCurrentPosition()));
                opMode.telemetry.addData("rightDT", rightDT.getCurrentPosition());

                opMode.telemetry.update();

                ErrorAmount = (Math.abs(((newleftDTTarget) - (leftDT.getCurrentPosition()))))
                        + (Math.abs(((newrightDTTarget) - (rightDT.getCurrentPosition()))) / COUNTS_PER_INCH);
                if (ErrorAmount < amountError) {
                    goodEnough = true;
                }
            }

            leftDT.setPower(0);
            rightDT.setPower(0);

            // Turn off RUN_TO_POSITION
            leftDT.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightDT.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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
    public void rangeDrive (double speed, double backDistance, double sideDistance, LinearOpMode opmode) {

        speed = Range.clip(Math.abs(speed), 0.0, 1.0);

        if(backDistance != -1) {
            while (backRange.getDistance(DistanceUnit.INCH) < backDistance){
                if (Math.abs(backRange.getDistance(DistanceUnit.INCH) - backDistance) > MAX_ACCEPTABLE_ERROR){
                    if (!rangeCheck(backRange,backDistance, opmode)){
                        break;
                    }
                }
                leftDT.setPower(-speed);
                rightDT.setPower(-speed);

                opmode.telemetry.addData("Sensor Front Distance: ", backRange.getDistance(DistanceUnit.INCH));
                opmode.telemetry.addData("Target Front Distance: ", backDistance);
                opmode.telemetry.addLine("Moving Backwards");
                opmode.telemetry.update();
            }
            while (backRange.getDistance(DistanceUnit.INCH) > backDistance){
                if (Math.abs(backRange.getDistance(DistanceUnit.INCH) - backDistance) > MAX_ACCEPTABLE_ERROR){
                    if (!rangeCheck(backRange,backDistance,opmode)){
                        break;
                    }
                }
                leftDT.setPower(speed);
                rightDT.setPower(speed);

                opmode.telemetry.addData("Sensor Front Distance: ", backRange.getDistance(DistanceUnit.INCH));
                opmode.telemetry.addData("Target Front Distance: ", backDistance);
                opmode.telemetry.addLine("Moving Forwards");
                opmode.telemetry.update();
            }
        }
        if (sideDistance != -1) {
            while (sideRange.getDistance(DistanceUnit.INCH) < sideDistance){
                if (Math.abs(sideRange.getDistance(DistanceUnit.INCH) - sideDistance) > MAX_ACCEPTABLE_ERROR){
                    if (!rangeCheck(sideRange,sideDistance,opmode)){
                        break;
                    }
                }
                leftDT.setPower(speed);
                rightDT.setPower(-speed);

                opmode.telemetry.addData("Sensor Left Distance: ", backRange.getDistance(DistanceUnit.INCH));
                opmode.telemetry.addData("Target Left Distance: ", backDistance);
                opmode.telemetry.addLine("Moving Right");
                opmode.telemetry.update();
            }
            while (sideRange.getDistance(DistanceUnit.INCH) > sideDistance){
                if (Math.abs(sideRange.getDistance(DistanceUnit.INCH) - sideDistance) > MAX_ACCEPTABLE_ERROR){
                    if (!rangeCheck(sideRange,sideDistance,opmode)){
                        break;
                    }
                }
                leftDT.setPower(-speed);
                rightDT.setPower(speed);

                opmode.telemetry.addData("Sensor Left Distance: ", backRange.getDistance(DistanceUnit.INCH));
                opmode.telemetry.addData("Target Left Distance: ", backDistance);
                opmode.telemetry.addLine("Moving Left");
                opmode.telemetry.update();
            }
        }
    }

    public void gyroTurn(double speed, double angle, LinearOpMode opmode) {

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

        // Display it for the driver.
        opmode.telemetry.addData("Target", "%5.2f", angle);
        opmode.telemetry.addData("Err/St", "%5.2f/%5.2f", error, steer);
        opmode.telemetry.addData("Speed ", "%5.2f:%5.2f", leftSpeed, rightSpeed);

        return onTarget;
    }

    public void lift(int level, LinearOpMode opmode){
        int levelOne       = 0;
        int levelTwo       = 3000;
        int levelThree     = 4000;
        int currentEncoder = lift.getCurrentPosition();
        int targetEncoder;

        if(level == 1){
            targetEncoder = levelOne;
        }
        else if(level == 2){
            targetEncoder = levelTwo;
        }
        else{
            targetEncoder = levelThree;
        }

        double power = 1;
        if(Math.abs(targetEncoder - currentEncoder) < 100){
            power = 0.3;
        }
        if(targetEncoder < currentEncoder){
            power = -power;
        }
        lift.setPower(power);
    }

}