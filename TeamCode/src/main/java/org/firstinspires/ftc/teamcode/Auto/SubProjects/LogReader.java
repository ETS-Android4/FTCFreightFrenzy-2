package org.firstinspires.ftc.teamcode.Auto.SubProjects;

import android.os.Environment;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;

@TeleOp(name = "Log Reader")
public class LogReader extends LinearOpMode {

    @Override
    public void runOpMode(){

        waitForStart();

        int angle = -1;
        try {
            File file = new File(Environment.getExternalStorageDirectory(), "number");
            BufferedReader br = new BufferedReader(new FileReader(file));
            String line;
            while ((line = br.readLine()) != null){
                telemetry.addLine("line is " + line);
            }
            telemetry.update();
            br.close();
        }
        catch(Exception e){
            telemetry.addLine("problem with i/o");
            telemetry.addLine(e.getLocalizedMessage());
            telemetry.update();
        }

        while (opModeIsActive()){

        }

    }

}