package org.firstinspires.ftc.teamcode.Autonomous;
//Declaring our imports
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

@Autonomous (name = "Bot-Lobster-Autonomous")

public class AutonomousTest extends LinearOpMode {

    //establishing motors

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;

    @Override
    public void runOpMode(){

        //assigning Volitle Variables Not needed?
        double horizontal;
        double vertical;
        double pivot;
        double FLeftMotor;
        double FRightMotor;
        double BLeftMotor;
        double BRightMotor;





        // assigning directions for motors
        leftFrontDrive = hardwareMap.get(DcMotor.class, "front left");
        leftBackDrive = hardwareMap.get(DcMotor.class, "back left");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "front right");
        rightBackDrive = hardwareMap.get(DcMotor.class, "back right");

        //Later implement opencv


        waitForStart();
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        if (opModeIsActive()){

            zeroMotors();

            //-functions-

            //Testing

            //move towards submersible
            moveY(.4,800);

            //assume 10 seconds (This is for placing specimen)
            sleep(2000);

            //park
            moveX(-.4,1000);






        }





        }
        //waittime is in miliseconds
        //Moves on the y axis (forward and back)
    private void moveY(double power, int waitTime) {
        leftFrontDrive.setPower(power);
        leftBackDrive.setPower(power);
        rightFrontDrive.setPower(power);
        rightBackDrive.setPower(power);
        sleep(waitTime);
        zeroMotors();


    }
    //Turns the bot (clockwise or counter)
    private void Turn(double power , int waitTime){
        leftFrontDrive.setPower(-power);
        rightFrontDrive.setPower(power);
        leftBackDrive.setPower(-power);
        rightBackDrive.setPower(power);
        sleep(waitTime);
        zeroMotors();
    }
    //moves the bot on the x axis (strafe: left or right)
    private void moveX(double power, int waitTime){
        leftFrontDrive.setPower( -power);
        leftBackDrive.setPower(power);
        rightFrontDrive.setPower(power);
        rightBackDrive.setPower( - power);
        sleep(waitTime);
        zeroMotors();
    }
    //sets motors to a set.power of 0.
    private void zeroMotors(){
        leftFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightBackDrive.setPower(0);
    }


    }


