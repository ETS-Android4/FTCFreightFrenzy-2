package org.firstinspires.ftc.teamcode.Auto.AltAutos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

//import org.firstinspires.ftc.teamcode.Auto.ContourDetector;
import org.firstinspires.ftc.teamcode.Base.MainBase;

@Disabled
@Autonomous(name="DT Test")
public class DriveTrainTest extends LinearOpMode{

    MainBase base = new MainBase();

    @Override
    public void runOpMode() throws InterruptedException {

        base.init(hardwareMap, this);

        base.rightDT.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        base.leftDT.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        base.rightDT.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        base.leftDT.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        base.gyro.resetZAxisIntegrator();

        //base.gyroDrive(1,50,50,0,0,0,this);

        base.encoderDrive(1,50,50,this);

    }
}