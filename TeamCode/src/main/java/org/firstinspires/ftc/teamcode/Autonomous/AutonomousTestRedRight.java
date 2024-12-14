package org.firstinspires.ftc.teamcode.Autonomous;
//Declaring our imports
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous (name = "Bot-Lobster-Autonomous")

public class AutonomousTestRedRight extends LinearOpMode {

    //establishing motors

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private CRServo HorizontalActuator = null;
    private CRServo VerticalLift = null;
    private Servo ScoopServo = null;

    private CRServo ServoWheel  = null;

    private Servo BlockServo = null;



    @Override
    public void runOpMode(){

        //assigning Double Variables (Not needed?)
        double horizontal;
        double vertical;
        double pivot;
        double FLeftMotor;
        double FRightMotor;
        double BLeftMotor;
        double BRightMotor;





        // assigning directions for motors and servos
        leftFrontDrive = hardwareMap.get(DcMotor.class, "front left");
        leftBackDrive = hardwareMap.get(DcMotor.class, "back left");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "front right");
        rightBackDrive = hardwareMap.get(DcMotor.class, "back right");

        HorizontalActuator = hardwareMap.get(CRServo.class, "horizontal acuator");
        VerticalLift = hardwareMap.get(CRServo.class, "vertical lift");
        ScoopServo = hardwareMap.get(Servo.class, "intake flippy");
        ServoWheel = hardwareMap.get(CRServo.class, "intake wheel");
        BlockServo = hardwareMap.get(Servo.class, "VLiftCLaw");



        //Later implement opencv


        waitForStart();
        //drivetrain motors
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Encoders motors
        HorizontalActuator.setDirection(CRServo.Direction.FORWARD);

        VerticalLift.setDirection(CRServo.Direction.FORWARD);


        //servos
        ScoopServo.setDirection(Servo.Direction.FORWARD);
        ServoWheel.setDirection(CRServo.Direction.FORWARD);
        BlockServo.setDirection(Servo.Direction.FORWARD);
        BlockServo.setPosition(Servo.MIN_POSITION);






        if (opModeIsActive()){


            zeroMotors();

            //-functions-

            //Testing

            //park
            moveX(.4,1000);








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
    //Function for Bucket Servo
    private void IntakeBucket (double position){
        ScoopServo.setPosition(position);
    }

    //Wheel for intake
    private void Intake (double power, int waitTime){
        ServoWheel.setPower(power);
        sleep(waitTime);

    }
    private void LiftY(double power, int waitTime){
        VerticalLift.setPower(power);
        sleep(waitTime);
    }
    private void Actuator (double power, int waitTime){
        HorizontalActuator.setPower(power);
        sleep(waitTime);
    }


    }


