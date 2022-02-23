package org.firstinspires.ftc.teamcode.Base;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class MainBase {

    // Total Motors: 5
    // Total Servos: 2
    public DcMotor leftDT      = null;
    public DcMotor rightDT     = null;
    public DcMotor leftDuck    = null;
    public DcMotor rightDuck   = null;
    public DcMotor lift        = null;
    public Servo   bucket      = null;
    public Servo   leftClaw    = null;


    // Total Sensors: 3
    public ModernRoboticsI2cRangeSensor backRange   = null;
    public ModernRoboticsI2cRangeSensor sideRange   = null;
    public ModernRoboticsI2cGyro        gyro        = null;


    static final double     COUNTS_PER_MOTOR_REV    = 386.3;
    static final double     DRIVE_GEAR_REDUCTION    = 1.0;
    static final double     WHEEL_DIAMETER_INCHES   = 2.5;
    static final double     EMPIRICAL_MULTIPLIER    = (30.0 / 17);
    public static final double     COUNTS_PER_INCH  = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION * EMPIRICAL_MULTIPLIER)
            / (WHEEL_DIAMETER_INCHES * 3.14159265);
    public double                  amountError        = 0.6;
    public static final double     P_TURN_COEFF       = 0.1;
    public static final double     HEADING_THRESHOLD  = 1.0;
    public static final double   MAX_ACCEPTABLE_ERROR = 10;
    public double                                 rpm = 0;
    final int LEVEL_ZERO              = 45;
    final int LEVEL_ONE               = 1700;
    final int LEVEL_TWO               = 3870;
    final int LEVEL_THREE             = 5000;
    final int LEVEL_CAP               = 5500;
    final int ACCEPTABLE_ERROR        = 0;
    final int TELEOP_ACCEPTABLE_ERROR = 30;

    //Utilized for liftAutoRED() method
    final int LEVEL_ZERO0              = 45;
    final int LEVEL_ONE1               = 2000;
    final int LEVEL_TWO2               = 3850;
    final int LEVEL_THREE3             = 6250;

    HardwareMap hwMap = null;

    public void init(HardwareMap ahwMap, OpMode opMode) {
        hwMap = ahwMap;

        leftDT    = hwMap.get(DcMotor.class, "leftDT");
        rightDT   = hwMap.get(DcMotor.class, "rightDT");
        leftDuck  = hwMap.get(DcMotor.class, "leftDuck");
        rightDuck = hwMap.get(DcMotor.class, "rightDuck");
        lift      = hwMap.get(DcMotor.class, "lift");
        bucket    = hwMap.get(Servo.class,"bucket");
        leftClaw  = hwMap.get(Servo.class, "leftClaw");

        backRange = hwMap.get(ModernRoboticsI2cRangeSensor.class,"backRange");
        backRange.initialize();

        sideRange  = hwMap.get(ModernRoboticsI2cRangeSensor.class,"sideRange");
        sideRange.initialize();

        gyro = hwMap.get(ModernRoboticsI2cGyro.class,"gyro");
        gyro.initialize();
        gyro.calibrate();
        while(gyro.isCalibrating());
        opMode.telemetry.addLine("Gyro Calibrated");
        opMode.telemetry.update();

        leftDT.setDirection(DcMotor.Direction.REVERSE);
        rightDT.setDirection(DcMotor.Direction.FORWARD);

        rightDT.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftDT.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDT.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftDT.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftDT.setPower(0);
        rightDT.setPower(0);
        leftDuck.setPower(0);
        rightDuck.setPower(0);
        lift.setPower(0);
        bucket.setPosition(1.0);
        leftClaw.setPosition(0.4);

        opMode.telemetry.addLine("Initialization Complete");
        opMode.telemetry.update();
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
        robotError = angle - gyro.getIntegratedZValue(); //Switch to gyro.getHeading() if issue arises.
        while (robotError > 180)  robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    public double getSteer (double error, double P_DRIVE_COEFF){
        return Range.clip(error * P_DRIVE_COEFF, -1, 1);
    }

    //Autonomous driving method that utilizes gyroscope to correct angle veer-offs during strafing
    public void gyroDrive (double speed, double distanceL, double distanceR, double angle,
                          double endFLPower, double endFRPower,
                          LinearOpMode opmode){

        int     newTLTarget;
        int     newTRTarget;
        int     moveCountsTL = (int)(distanceL * MainBase.COUNTS_PER_INCH);
        int     moveCountsTR = (int)(distanceR * MainBase.COUNTS_PER_INCH);

        double  max;
        double  error;
        double  speedL;
        double  speedR;
        double  ErrorAmount;
        boolean goodEnough = false;

        // Ensure that the Op-mode is still active
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

                ErrorAmount = (Math.abs(newTLTarget - leftDT.getCurrentPosition())
                        + Math.abs(newTRTarget - rightDT.getCurrentPosition())) / COUNTS_PER_INCH;
                if (ErrorAmount < amountError) {
                    goodEnough = true;
                }

                leftDT.setPower(speedL);
                rightDT.setPower(speedR);

                // Display drive status for the driver.
                opmode.telemetry.addData("Err/St",  "%5.1f/%5.1f",  error, steer);
                opmode.telemetry.addData("Target",  "%7d:%7d", newTLTarget, newTRTarget);
                opmode.telemetry.addData("Actual",  "%7d:%7d", leftDT.getCurrentPosition(),
                        rightDT.getCurrentPosition());
                opmode.telemetry.addData("Speed",   "%5.2f:%5.2f", speedL, speedR);

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
                    ((leftDT.isBusy() || rightDT.isBusy()) && !goodEnough)) {

                // Display it for the driver.
                opMode.telemetry.addData("Path1", "Running to %7d :%7d", newleftDTTarget, newrightDTTarget);
                opMode.telemetry.addData("Path2", "Running at %7d :%7d",

                        leftDT.getCurrentPosition(),
                        rightDT.getCurrentPosition());
                opMode.telemetry.addData("leftDT", leftDT.getCurrentPosition());
                opMode.telemetry.addData("rightDT", rightDT.getCurrentPosition());

                ErrorAmount = (Math.abs(newleftDTTarget - leftDT.getCurrentPosition())
                             + Math.abs(newrightDTTarget - rightDT.getCurrentPosition())) / COUNTS_PER_INCH;
                if (ErrorAmount < amountError) {
                    goodEnough = true;
                }

                opMode.telemetry.addData("Error Amount", ErrorAmount);
                opMode.telemetry.update();
            }

            leftDT.setPower(0);
            rightDT.setPower(0);

            // Turn off RUN_TO_POSITION
            leftDT.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightDT.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }
    }

    //Method overload to add in end powers of encoderDrive()
    public void encoderDrive(double speed, double leftDTInches, double rightDTInches,
                             double endLPower, double endRPower, LinearOpMode opMode){
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
                    ((leftDT.isBusy() || rightDT.isBusy()) && !goodEnough)) {

                // Display it for the driver.
                opMode.telemetry.addData("Path1", "Running to %7d :%7d", newleftDTTarget, newrightDTTarget);
                opMode.telemetry.addData("Path2", "Running at %7d :%7d",

                        leftDT.getCurrentPosition(),
                        rightDT.getCurrentPosition());
                opMode.telemetry.addData("leftDT", leftDT.getCurrentPosition());
                opMode.telemetry.addData("rightDT", rightDT.getCurrentPosition());

                ErrorAmount = (Math.abs(newleftDTTarget - leftDT.getCurrentPosition())
                        + Math.abs(newrightDTTarget - rightDT.getCurrentPosition())) / COUNTS_PER_INCH;
                if (ErrorAmount < amountError) {
                    goodEnough = true;
                }

                opMode.telemetry.addData("Error Amount", ErrorAmount);
                opMode.telemetry.update();
            }

            leftDT.setPower(endLPower);
            rightDT.setPower(endRPower);

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
                    if (!rangeCheck(backRange, backDistance, opmode)){
                        break;
                    }
                }
                leftDT.setPower(-speed);
                rightDT.setPower(-speed);

                opmode.telemetry.addData("Sensor Back Distance: ", backRange.getDistance(DistanceUnit.INCH));
                opmode.telemetry.addData("Target Back Distance: ", backDistance);
                opmode.telemetry.addLine("Moving Backwards");
                opmode.telemetry.update();
            }
            while (backRange.getDistance(DistanceUnit.INCH) > backDistance){
                if (Math.abs(backRange.getDistance(DistanceUnit.INCH) - backDistance) > MAX_ACCEPTABLE_ERROR){
                    if (!rangeCheck(backRange, backDistance, opmode)){
                        break;
                    }
                }
                leftDT.setPower(speed);
                rightDT.setPower(speed);

                opmode.telemetry.addData("Sensor Back Distance: ", backRange.getDistance(DistanceUnit.INCH));
                opmode.telemetry.addData("Target Back Distance: ", backDistance);
                opmode.telemetry.addLine("Moving Forwards");
                opmode.telemetry.update();
            }
        }
        if (sideDistance != -1) {
            while (sideRange.getDistance(DistanceUnit.INCH) < sideDistance){
                if (Math.abs(sideRange.getDistance(DistanceUnit.INCH) - sideDistance) > MAX_ACCEPTABLE_ERROR){
                    if (!rangeCheck(sideRange, sideDistance, opmode)){
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
                    if (!rangeCheck(sideRange, sideDistance, opmode)){
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

    //Utilization of Gyro to turn robot
    public void gyroTurn(double speed, double angle, LinearOpMode opmode) {

        // keep looping while we are still active, and not on heading.
        while (opmode.opModeIsActive() && !onHeading(speed, angle, P_TURN_COEFF, opmode)) {
            // Update telemetry & Allow time for other processes to run.
            opmode.telemetry.update();
        }
    }

    //Custom sleep method
    public void sleepV2(double wait, LinearOpMode opmode) {
        double finalTime = opmode.time + wait;
        while(finalTime > opmode.time){
            opmode.telemetry.update();
        }
    }

    boolean onHeading(double speed, double angle, double PCoeff, LinearOpMode opmode) {
        double   error;
        double   steer;
        boolean  onTarget = false;
        double leftSpeed;
        double rightSpeed;

        // determine turn power based on +/- error
        error = getError(-angle);

        if (Math.abs(error) <= HEADING_THRESHOLD) {
            steer = 0.0;
            leftSpeed  = 0.0;
            rightSpeed = 0.0;
            onTarget = true;
        }
        else {
            steer = getSteer(error, PCoeff);
            /*rightSpeed  = speed * steer;
            leftSpeed   = -rightSpeed;*/

            leftSpeed  = speed * steer;
            rightSpeed   = -leftSpeed;
        }

        // Send desired speeds to motors.
        leftDT.setPower(leftSpeed);
        rightDT.setPower(rightSpeed);

        // Display it for the driver.
        opmode.telemetry.addData("Target", "%5.2f", angle);
        opmode.telemetry.addData("Err/St", "%5.2f/%5.2f", error, steer);
        opmode.telemetry.addData("Speed ", "%5.2f:%5.2f", leftSpeed, rightSpeed);
        opmode.telemetry.addData("current Angle", gyro.getHeading());
        opmode.telemetry.addData("current Angle Z", gyro.getIntegratedZValue());

        return onTarget;
    }

    //Automated lift method for TeleOp (Levels 1 & 2 NOT in use)
    public void lift(int level, LinearOpMode opmode){

        int currentEncoder = lift.getCurrentPosition();
        int targetEncoder;

        if (level == 0){
            targetEncoder = LEVEL_ZERO;
        }
        else if (level == 1){
            targetEncoder = LEVEL_ONE;
        }
        else if (level == 2){
            targetEncoder = LEVEL_TWO;
        }
        else if (level == 3){
            targetEncoder = LEVEL_THREE;
        }
        else {
            targetEncoder = LEVEL_CAP;
        }

        double power = 1;
        if(Math.abs(targetEncoder - currentEncoder) < 75) {
            power = 0.7;
        }
        if(Math.abs(targetEncoder - currentEncoder) < TELEOP_ACCEPTABLE_ERROR) {
            power = 0;
        }
        if(targetEncoder < currentEncoder){
            power = -power;
        }
        lift.setPower(power);
    }

    //Lift method for Autonomous w/ WAIT
    public void liftAuto(int level, boolean liftWait, LinearOpMode opMode) {
        if (opMode.opModeIsActive()) {


            int targetEncoder = 0;
            //Setting target level of lift
            if (level == 0) {
                targetEncoder = LEVEL_ZERO;
            } else if (level == 1) {
                targetEncoder = LEVEL_ONE;
            } else if (level == 2) {
                targetEncoder = LEVEL_TWO;
            } else if (level == 3) {
                targetEncoder = LEVEL_THREE;
            }

            lift.setTargetPosition(targetEncoder);
            // Turn ON RUN_TO_POSITION
            lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            // Turn ON lift
            lift.setPower(1);

            boolean goodEnough = false;
            while (liftWait && opMode.opModeIsActive() && (lift.isBusy()) && !goodEnough) {

                double ErrorAmount = Math.abs(targetEncoder - lift.getCurrentPosition());
                if (ErrorAmount < ACCEPTABLE_ERROR) {
                    goodEnough = true;
                }

                opMode.telemetry.addData("Error Amount", ErrorAmount);
                opMode.telemetry.addData("Good Enough: ", goodEnough);
               // opMode.telemetry.update();
            }
        }
    }

    //Lift method for Autonomous w/ NO WAIT
    public void liftAuto(int level, LinearOpMode opMode){
        if (opMode.opModeIsActive()) {

            int targetEncoder = 0;

            //Setting target level of lift
            if (level == 0) {
                targetEncoder = LEVEL_ZERO;
            } else if (level == 1) {
                targetEncoder = LEVEL_ONE;
            } else if (level == 2) {
                targetEncoder = LEVEL_TWO;
            } else if (level == 3) {
                targetEncoder = LEVEL_THREE;
            }

            lift.setTargetPosition(targetEncoder);
            // Turn ON RUN_TO_POSITION
            lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            // Turn ON lift
            lift.setPower(1);

            boolean goodEnough = false;
            while (opMode.opModeIsActive() && (lift.isBusy()) && !goodEnough) {

                double ErrorAmount = Math.abs(targetEncoder - lift.getCurrentPosition());
                if (ErrorAmount < ACCEPTABLE_ERROR) {
                    goodEnough = true;
                }

                opMode.telemetry.addData("Error Amount", ErrorAmount);
                opMode.telemetry.addData("Good Enough: ", goodEnough);
                // opMode.telemetry.update();
            }

            // Turn off lift power
            lift.setPower(0);

            // Turn off RUN_TO_POSITION
            lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    //Lift method for Autonomous w/ NO WAIT (Utilized for RED-Specific AUTO)
    public void liftAutoRED(int level, boolean liftWait, LinearOpMode opMode) {
        if (opMode.opModeIsActive()) {

            int targetEncoder = 0;

            //Setting target level of lift
            if (level == 0) {
                targetEncoder = LEVEL_ZERO0;
            } else if (level == 1) {
                targetEncoder = LEVEL_ONE1;
            } else if (level == 2) {
                targetEncoder = LEVEL_TWO2;
            } else if (level == 3) {
                targetEncoder = LEVEL_THREE3;
            }

            lift.setTargetPosition(targetEncoder);
            // Turn ON RUN_TO_POSITION
            lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            // Turn ON lift
            lift.setPower(1);

            boolean goodEnough = false;
            while (liftWait && opMode.opModeIsActive() && (lift.isBusy()) && !goodEnough) {

                double ErrorAmount = Math.abs(targetEncoder - lift.getCurrentPosition());
                if (ErrorAmount < ACCEPTABLE_ERROR) {
                    goodEnough = true;
                }

                opMode.telemetry.addData("Error Amount", ErrorAmount);
                opMode.telemetry.addData("Good Enough: ", goodEnough);
                // opMode.telemetry.update();
            }
        }
    }
}
