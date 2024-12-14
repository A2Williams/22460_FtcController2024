/* Copyright (c) 2021 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.TeleOpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
/*
 * This file contains an example of a Linear "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode is executed.
 *
 * This particular OpMode illustrates driving a 4-motor Omni-Directional (or Holonomic) robot.
 * This code will work with either a Mecanum-Drive or an X-Drive train.
 * Both of these drives are illustrated at https://gm0.org/en/latest/docs/robot-design/drivetrains/holonomic.html
 * Note that a Mecanum drive must display an X roller-pattern when viewed from above.
 *
 * Also note that it is critical to set the correct rotation direction for each motor.  See details below.
 *
 * Holonomic drives provide the ability for the robot to move in three axes (directions) simultaneously.
 * Each motion axis is controlled by one Joystick axis.
 *
 * 1) Axial:    Driving forward and backward               Left-joystick Forward/Backward
 * 2) Lateral:  Strafing right and left                     Left-joystick Right and Left
 * 3) Yaw:      Rotating Clockwise and counter clockwise    Right-joystick Right and Left
 *
 * This code is written assuming that the right-side motors need to be reversed for the robot to drive forward.
 * When you first test your robot, if it moves backward when you push the left stick forward, then you must flip
 * the direction of all 4 motors (see code below).
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@TeleOp(name="Basic: basic meccanum driveUsingEncoders", group="test OpMode")
public class MeccanumDriveTeleopUsing_Encoders extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;


    private DcMotor HorizontalActuator = null;
    private DcMotor VerticalLift = null;
    private Servo ScoopServo = null;

    private Servo ScoopServo2 = null;

    private CRServo ServoWheel  = null;
    private Servo BlockServo = null;

    private CRServo BucketV  = null;

    // the value for percsion mode
    private final double PERCISION_VALUE = 0.5;

    @Override
    public void runOpMode() {


        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        leftFrontDrive = hardwareMap.get(DcMotor.class, "front left");
        leftBackDrive = hardwareMap.get(DcMotor.class, "back left");

        rightFrontDrive = hardwareMap.get(DcMotor.class, "front right");
        rightBackDrive = hardwareMap.get(DcMotor.class, "back right");

        HorizontalActuator = hardwareMap.get(DcMotor.class, "horizontal acuator");
        VerticalLift = hardwareMap.get(DcMotor.class, "vertical lift");

        ScoopServo = hardwareMap.get(Servo.class, "intake flippy");
        ScoopServo2 = hardwareMap.get(Servo.class, "intake flippy2");

        ServoWheel = hardwareMap.get(CRServo.class, "intake wheel");
        BucketV = hardwareMap.get(CRServo.class, "Bucket4Lift");
        BlockServo = hardwareMap.get(Servo.class, "VLiftCLaw");
        // ########################################################################################
        // !!!            IMPORTANT Drive Information. Test your motor directions.            !!!!!
        // ########################################################################################
        // Most robots need the motors on one side to be reversed to drive forward.
        // The motor reversals shown here are for a "direct drive" robot (the wheels turn the same direction as the motor shaft)
        // If your robot has additional gear reductions or uses a right-angled drive, it's important to ensure
        // that your motors are turning in the correct direction.  So, start out with the reversals here, BUT
        // when you first test your robot, push the left joystick forward and observe the direction the wheels turn.
        // Reverse the direction (flip FORWARD <-> REVERSE ) of any wheel that runs backward
        // Keep testing until ALL the wheels move the robot forward when you push the left joystick forward.

        //Drive Train Motors
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        //Encodered Motors

        HorizontalActuator.setDirection(DcMotor.Direction.FORWARD);
        //

        HorizontalActuator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        VerticalLift.setDirection(DcMotor.Direction.FORWARD);


        VerticalLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        //Servos
        ScoopServo.setDirection(Servo.Direction.FORWARD);
        ScoopServo2.setDirection(Servo.Direction.FORWARD);
        ScoopServo2.setPosition(Servo.MIN_POSITION);

        BucketV.setDirection(DcMotorSimple.Direction.FORWARD);
        ServoWheel.setDirection(CRServo.Direction.FORWARD);
        BlockServo.setDirection(Servo.Direction.FORWARD);
        BlockServo.setPosition(Servo.MIN_POSITION);


        //lift motors
       // lift1move.setDirection(DcMotorSimple.Direction.FORWARD);
        //lift2move.setDirection(DcMotorSimple.Direction.FORWARD);

        //Sets servo var
        double position = 0.0;


        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            //Pulses Per Revolution on the GOBILDA Yellow Jacket 5203 Model Motors
            double PPR = (384.5);

            //Ticks variable tells the exact location of motor through encoders
            int TicksH = HorizontalActuator.getCurrentPosition();
            int TicksV = VerticalLift.getCurrentPosition();

            double RevolutionsH = TicksH/PPR;
            double RevolutionsV = TicksV/PPR;


            float lift = gamepad2.right_stick_y;
            float XArm = gamepad2.left_stick_y;
            float Clockwise = gamepad2.right_trigger;
            float CClockWise = gamepad2.left_trigger;


            double max;
            double percsion = 1.0;
            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double yAxis = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double xAxis = gamepad1.left_stick_x;
            double zAxis = gamepad1.right_stick_x;
            if (gamepad1.right_bumper) {
                percsion = PERCISION_VALUE;
            }

            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            double leftFrontPower = percsion * (yAxis + xAxis + zAxis);
            double rightFrontPower = percsion * (yAxis - xAxis - zAxis);
            double leftBackPower = percsion * (yAxis - xAxis + zAxis);
            double rightBackPower = percsion * (yAxis + xAxis - zAxis);


            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));


            if (max > 1.0) {
                leftFrontPower /= max;
                rightFrontPower /= max;
                leftBackPower /= max;
                rightBackPower /= max;
            }

            // This is test code:
            //
            // Uncomment the following code to test your motor directions.
            // Each button should make the corresponding motor run FORWARD.
            //   1) First get all the motors to take to correct positions on the robot
            //      by adjusting your Robot Configuration if necessary.
            //   2) Then make sure they run in the correct direction by modifying the
            //      the setDirection() calls above.
            // Once the correct motors move in the correct direction re-comment this code.

            /*
            leftFrontPower  = gamepad1.x ? 1.0 : 0.0;  // X gamepad
            leftBackPower   = gamepad1.a ? 1.0 : 0.0;  // A gamepad
            rightFrontPower = gamepad1.y ? 1.0 : 0.0;  // Y gamepad
            rightBackPower  = gamepad1.b ? 1.0 : 0.0;  // B gamepad
            */

            // Send calculated power to wheels
            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightBackDrive.setPower(rightBackPower);

            //Makes sure that the motors go past ticks 0, and 8250 for the actuator
            if(XArm > 0 && TicksH > 0) {

                HorizontalActuator.setPower(gamepad2.left_stick_y);

            }

            else if(XArm < 0 && TicksH < 8250) {

                HorizontalActuator.setPower(gamepad2.left_stick_y);

            }

            else {

                HorizontalActuator.setPower(0);

            }
            //Will Extend if left stick is pushed forward and Shorten if pushed backward.

            // Limits set on the vertical lift so it does not go past ticks 0 and 7800
            if(lift > 0 && TicksV > 0) {

                VerticalLift.setPower(-lift);

            }

            else if(lift < 0 && TicksV < 7800) {

                VerticalLift.setPower(-lift);

            }

            else {

                VerticalLift.setPower(0);

            }


            // Blue Scooper
            if(gamepad2.dpad_up) {
                ScoopServo.setPosition(0.7);
                ScoopServo2.setPosition(0.7);
            }

            if(gamepad2.dpad_down) {
                ScoopServo.setPosition(0);
                ScoopServo2.setPosition(0);

            }
            if(gamepad2.dpad_right){
                ScoopServo.setPosition(0.5);
                ScoopServo2.setPosition(0.5);

            }

            ServoWheel.setPower(Clockwise);
            ServoWheel.setPower(-CClockWise);

            //Blue sample holder
            if(gamepad2.right_bumper) {
                BlockServo.setPosition(0.8);
            }

            if(gamepad2.left_bumper) {
                BlockServo.setPosition(0);
            }




            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
            telemetry.addData("servo1 position", "%4.2f", ScoopServo.getPosition());
            telemetry.addData("servo2 Position", "%4.2f", ScoopServo2.getPosition());
            telemetry.addData("EncoderPositionH", TicksH);
            telemetry.addData("EncoderPositionV", TicksV);
            telemetry.addData("RevolutionsH", RevolutionsH);
            telemetry.addData("RevolutionsV", RevolutionsV);
            telemetry.update();
        }

    }}
