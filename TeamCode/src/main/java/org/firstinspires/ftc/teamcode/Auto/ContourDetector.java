package org.firstinspires.ftc.teamcode.Auto;

import com.arcrobotics.ftclib.vision.UGContourRingPipeline;
import com.arcrobotics.ftclib.vision.UGContourRingDetector;
import com.arcrobotics.ftclib.vision.UGContourRingDetector;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.openftc.easyopencv.OpenCvCameraRotation;

public class ContourDetector{

    public UGContourRingDetector ringDetector;

    public ContourDetector(HardwareMap hardwareMap, Telemetry telemetry){

        UGContourRingDetector.PipelineConfiguration.setCAMERA_WIDTH(320);
        UGContourRingDetector.PipelineConfiguration.setCAMERA_HEIGHT(240);

        UGContourRingDetector.PipelineConfiguration.setHORIZON(100);

        UGContourRingDetector.PipelineConfiguration.setCAMERA_ORIENTATION(OpenCvCameraRotation.UPRIGHT);

        // Creates a webcam detector with telemetry debugging
        ringDetector = new UGContourRingDetector(hardwareMap, "sideCAM", telemetry, true);

        ringDetector.init();
    }
}
