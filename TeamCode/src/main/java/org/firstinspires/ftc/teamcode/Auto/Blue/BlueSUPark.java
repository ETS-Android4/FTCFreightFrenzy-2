package org.firstinspires.ftc.teamcode.Auto.Blue;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Auto.Detection.ObjectDetector;
import org.firstinspires.ftc.teamcode.Base.MainBase;
import org.firstinspires.ftc.teamcode.Base.Variables;


@Autonomous(name= "BLUE-SU PARKING")
public class BlueSUPark extends LinearOpMode{

    MainBase base = new MainBase();
    Variables var = new Variables();

    @Override
    public void runOpMode() throws InterruptedException {


        base.init(hardwareMap,this);

        base.rightDT.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        base.leftDT.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        base.rightDT.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        base.leftDT.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        waitForStart();

        //Blue autonomous: Delivers Duck and Parks in Storage Unit
        //Position: Back facing Carousel (Back 10 degrees from wall.)
        //-13 in, 50r, 32 in, 140r, 8 in
        base.encoderDrive(0.5,-20,-20,this); // drive to Carousel
        base.leftDuck.setPower(.42); // spin it
        sleep(2500); // for 2.5 sec.
        base.leftDuck.setPower(0);
        base.gyroTurn(.5,110,this); //rotate front towards SU
        base.encoderDrive(.5,21.5,21.5,this);// drive into SU
        base.gyroTurn(.5,99,this);
        telemetry.addData("Parked in BLUE SU :)","");
        telemetry.update();

    }
}
