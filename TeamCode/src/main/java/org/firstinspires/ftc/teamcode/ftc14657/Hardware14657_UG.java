package org.firstinspires.ftc.teamcode.ftc14657;

/* Copyright (c) 2017 FIRST. All rights reserved.
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

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a RoboWarriors 14657.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Front Left  drive motor:         "FLeftMotor"
 * Motor channel:  Front Right  drive motor:        "FRightMotor"
 * Motor channel:  Back Left  drive motor:          "BLeftMotor"
 * Motor channel:  Back Right  drive motor:         "BRightMotor"
 * Motor channel:  Intake Arm Lifter motor:         "IntakeArmLifter"
 * Motor channel:  Left Intake motor:               "leftIntake"
 * Motor channel:  Right Intake motor:              "rightIntake"
 * Motor channel:  Linear Slider motor:              "LinearSlider"
 *
 * Servo channel: H2 P2  Servo to push stone:             "StonePusher"
 * Servo channel: H2 P1  Front Servo to move skystone:    "FStoneMover"
 * Servo channel: H2 P0 Back Servo to move skystone:     "BStoneMover"
 * Servo channel: H2 P4  Servo to push stone:             "CapstoneMover"
 * Servo channel: H3 P1 Servo to move foundation:        "FoundationMover"
 * Servo channel: H3 P2 Servo to grab stone:             "Gripper"
 * Servo channel: H3 P3 Servo to move gripper:           "Gripper2"
 * Servo channel: H3 P4 Servo to move gripper:           "GripperMover"
 */
public class Hardware14657_UG {

    /* Public OpMode members. */
//    public DcMotor fLeft = null;
//    public DcMotor fRight = null;
//    public DcMotor bLeft = null;
//    public DcMotor bRight = null;

//    public DcMotor shooter = null;
    public DcMotorEx shooter = null;
    public DcMotor wobble = null;
//    public DcMotor collection = null;
    public DcMotorEx collection = null;
    public DcMotor lift = null;
    // public DcMotor leftIntake = null;
    // public DcMotor LinearSlider = null;

    //    public Servo stonePusher = null;
    public Servo grab = null;
    public Servo trigger = null;
    public Servo grab2 = null;
//    public CRServo flip = null;
    public Servo wobble2 = null;



    // servo move positions
    public float grab_openPos = 0f;
    public float grab_closePos = 0.7f;

    public float grab2_openPos = 0.1f;
    public float grab2_closePos = 0.5f;//0.6f;

    public float wobble2_liftPos = 0.35f;
    public float wobble2_dropPos = 0.9f;
    //
    //     public float push_TargetPos = 1f;
    //     public float push_ResetPos = 0f;

    public float trigger_closePos = 0f;
    public float trigger_openPos = 0.5f;


    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public Hardware14657_UG(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

//        // Define and Initialize Motors
//        fLeft = ahwMap.dcMotor.get("FLeftMotor");
//        fRight = ahwMap.dcMotor.get("FRightMotor");
//        bLeft = ahwMap.dcMotor.get("BLeftMotor");
//        bRight = ahwMap.dcMotor.get("BRightMotor");
//        collection  = ahwMap.dcMotor.get("Collection");

        collection  = ahwMap.get(DcMotorEx.class, "Collection");
        collection.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wobble =ahwMap.dcMotor.get("WobbleGoal");
        lift = ahwMap.dcMotor.get("Lift");
//        shooter =ahwMap.dcMotor.get("Shooter");
        shooter = ahwMap.get(DcMotorEx.class, "Shooter");
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

//
//        fLeft.setDirection(DcMotor.Direction.REVERSE);
//        fRight.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE
//        bLeft.setDirection(DcMotor.Direction.REVERSE);
//        bRight.setDirection(DcMotor.Direction.FORWARD);// Set to REVERSE
//
//        fLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        fRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        bLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        bRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        // LinearSlider.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        //wobble.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        //        collection.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//
//        // Set all motors to zero power
//        fLeft.setPower(0);
//        fRight.setPower(0);
//        bLeft.setPower(0);
//        bRight.setPower(0);
//
//        // Set all motors to run without encoders.
//        // May want to use RUN_USING_ENCODERS if encoders are installed.
//        fLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        fRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        bLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        bRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        // Servo set up
        grab = ahwMap.get(Servo.class, "Grab");
        trigger = ahwMap.get(Servo.class, "Trigger");
//        flip = ahwMap.get(CRServo.class, "Flip");
        grab2 = ahwMap.get(Servo.class, "Grab2");
        wobble2 = ahwMap.get(Servo.class, "Wobble2");

    }


}

