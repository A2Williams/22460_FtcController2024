package org.firstinspires.ftc.teamcode.CameraCode;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@TeleOp
public class Vison extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        AprilTagProcessor tagProcesor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .build();

        VisionPortal visionPortal = new VisionPortal.Builder()
                .addProcessor(tagProcesor)
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1" ))
                .setCameraResolution(new Size(640,480))
                .build();
        waitForStart();

        while (!isStopRequested() && opModeIsActive()) {
            if (tagProcesor.getDetections().size() > 0){
                AprilTagDetection tag = tagProcesor.getDetections().get(0);
                telemetry.addData("x", tag.ftcPose.x);
                telemetry.addData("y", tag.ftcPose.y);
                telemetry.addData("roll", tag.ftcPose.roll);
                telemetry.addData("pitch", tag.ftcPose.pitch);
                telemetry.addData("yaw", tag.ftcPose.yaw);
            }
            telemetry.update();

        }
    }
}
