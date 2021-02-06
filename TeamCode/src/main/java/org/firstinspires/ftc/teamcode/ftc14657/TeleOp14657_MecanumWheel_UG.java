package org.firstinspires.ftc.teamcode.ftc14657;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.opmode.DriveVelocityPIDTuner;

@TeleOp(name="TeleOp UG OnBot", group="Linear Opmode")
public class TeleOp14657_MecanumWheel_UG extends  Auto14657_Base_UG {

    private ElapsedTime runtime = new ElapsedTime();
    double shooterPower;

    Trajectory traj1;
    Trajectory traj2;
    boolean was_dpad_left = false;
    boolean was_dpad_right = false;
    double drive_power = 0.85;


    enum Mode {
        DRIVER_MODE,
        SHOOTING_MODE,
        POWERSHOT_MODE
    }

    public void init(HardwareMap ahwMap) {
//        robot.init(hardwareMap);
        robot.init(hardwareMap);
        drive = new SampleMecanumDrive(hardwareMap);



    }
    @Override
    public void runOpMode() throws InterruptedException{
        // set default mode
        Mode mode = Mode.DRIVER_MODE;

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        init(hardwareMap);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();
        robot.shooter.setDirection(DcMotor.Direction.FORWARD);
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            telemetry.addData("mode", mode);

            switch (mode)
            {
                case DRIVER_MODE:
                    drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                    if (gamepad2.a) {
                        mode = Mode.SHOOTING_MODE;
                        runtime.reset();
                    } else if (gamepad2.b) {
                        mode = Mode.POWERSHOT_MODE;
                        runtime.reset();
                    }

                    //Gamepad2 stuff

                    if (gamepad2.right_trigger > 0.1) {
                        robot.lift.setPower(-gamepad2.right_trigger);
                        robot.collection.setPower(gamepad2.right_trigger);
                    } else if (gamepad2.left_trigger > 0.1) {
                        robot.lift.setPower(gamepad2.left_trigger);
                        robot.collection.setPower(-gamepad2.left_trigger);
                    } else {
                        robot.lift.setPower(0);
                        robot.collection.setPower(0);
                    }
                    double drive2 = -gamepad2.right_stick_y;
                    if (gamepad2.left_bumper) {
//                shooterPower    = Range.clip(drive2, -0.7, 0.7) ;
                        shooterPower = Velocity_Powershot;
                    } else if(gamepad2.right_bumper){
//                shooterPower    = Range.clip(drive2, -0.95, 0.95) ;
                        shooterPower = Velocity_HighGoal;
                    } else {
                        shooterPower = 0;
                    }
                    robot.shooter.setVelocity(shooterPower);

                    if (gamepad2.dpad_down) {
//                robot.trigger.setPosition(robot.trigger_openPos);
                        runServo_TriggerOpen(0);
                    }else{
//                robot.trigger.setPosition(robot.trigger_closePos);
                        runServo_TriggerClose(0);
                    }
                    //TODO: change to gamepad2.dpad_left and dpad_right
                    if (gamepad2.x) {
                        robot.flip.setPower(1);
                    }else if(gamepad2.y) {
                        robot.flip.setPower(-1);
                    }else{
                        robot.flip.setPower(0);

                    }

                    break;
                case SHOOTING_MODE:
                    if (gamepad2.x)
                    {
                        mode = Mode.DRIVER_MODE;
                        drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    }
                    if (gamepad2.a)
                    {
                        int milisec_Revert = 200;
                        int milesec_MotorRotate = 2000 + milisec_Revert;
//                        int milisec_Shooter = 1200 + milesec_MotorRotate;

                        // revert the collection
                        if (runtime.milliseconds() <= milisec_Revert)
                        {
                            robot.collection.setPower(-0.5);
                            robot.shooter.setVelocity(-100);
                            sleep(150);
                        }
                        else if (runtime.milliseconds() > milisec_Revert && runtime.milliseconds() <= milesec_MotorRotate )
                        {

                            robot.collection.setPower(0);
                            // turn on shooter high speed motor
                            robot.shooter.setVelocity(Velocity_HighGoal);
//                            robot.shooter.setVelocity(1950);
//                        } else if (runtime.milliseconds() > milesec_MotorRotate  && runtime.milliseconds() <= milisec_Shooter ) {
                        }
                        else if (runtime.milliseconds() > milesec_MotorRotate )
                        {
                            // open the trigger, once it is open start shooting
                            if (gamepad2.right_trigger > 0.1) {
                                runServo_TriggerOpen(0);
                                sleep(300);

                                robot.collection.setPower(1);
                                robot.lift.setPower(-1);

                            }
                        }
                    } else {
                        // release button A, reset shooter motor/ close trigger
                        robot.shooter.setVelocity(0);
                        robot.collection.setPower(0);
                        robot.lift.setPower(0);
                        runServo_TriggerClose(0);
                        // reset runtime
                        runtime.reset();
                    }
                    break;
                case POWERSHOT_MODE:
                    if (gamepad2.x)
                    {
                        mode = Mode.DRIVER_MODE;
                        drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    }
                    if (gamepad2.b)
                    {
                        int milisec_Revert = 200;
                        int milesec_MotorRotate = 2000 + milisec_Revert;
    //                        int milisec_Shooter = 1200 + milesec_MotorRotate;

                        // revert the collection
                        if (runtime.milliseconds() <= milisec_Revert)
                        {
                            robot.collection.setPower(-0.5);
                            robot.shooter.setVelocity(-100);
                            sleep(150);
                        }
                        else if (runtime.milliseconds() > milisec_Revert && runtime.milliseconds() <= milesec_MotorRotate )
                        {
                            robot.collection.setPower(0);
                            // turn on shooter high speed motor
                            robot.shooter.setVelocity(Velocity_Powershot);
                        }
                        else if (runtime.milliseconds() > milesec_MotorRotate )
                        {
                            if (gamepad2.right_trigger > 0.1)
                            {
                                runServo_TriggerOpen(0);
                                sleep(300);

                                robot.collection.setPower(0.5);
                                robot.lift.setPower(-0.5);
                                sleep(200);
                                robot.lift.setPower(0);
                                robot.collection.setPower(0);
//                                runServo_TriggerClose(0);
//                                sleep(300);
//                                sleep(200);
                            }

                            if (gamepad2.dpad_left)
                            {
                                if(!was_dpad_left) {
                                    traj1 = drive.trajectoryBuilder(new Pose2d(), true)
                                            .lineToLinearHeading(new Pose2d(0, 7, Math.toRadians(0)))
                                            .build();
                                    drive.followTrajectory(traj1);
                                    was_dpad_left = true;
                                } else {
                                    traj1 = drive.trajectoryBuilder(traj1.end(), true)
                                            .lineToLinearHeading(new Pose2d(0, 14, Math.toRadians(0)))
                                            .build();
                                    drive.followTrajectory(traj1);
                                }

                            }

//                            if (gamepad2.dpad_right)
//                            {
//                                if(!was_dpad_right) {
//                                    traj2 = drive.trajectoryBuilder(new Pose2d(), true)
//                                            .strafeRight(7)
//                                            .build();
//                                    drive.followTrajectory(traj2);
//                                    was_dpad_right = true;
//                                } else {
//                                    traj2 = drive.trajectoryBuilder(traj2.end(), true)
//                                            .strafeRight(7)
//                                            .build();
//                                    drive.followTrajectory(traj2);
//                                }
//                            }

                        }

                    } else {
                        robot.shooter.setVelocity(0);
                        robot.collection.setPower(0);
                        robot.lift.setPower(0);
                        runServo_TriggerClose(0);
                        // reset runtime
                        runtime.reset();
                    }
                    break;

            }

            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y * drive_power,
                            -gamepad1.left_stick_x * drive_power,
                            -gamepad1.right_stick_x * drive_power
                    )
            );

            drive.update();

            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.addData("Velocity", robot.shooter.getVelocity());
            telemetry.update();

            // this was copy and pasted

            if (gamepad1.left_bumper) {
//                robot.grab.setPosition(1);
                openServo_Grab(0, false);
            } else if (gamepad1.right_bumper) {
                closeServo_Grab(0);
//                robot.grab.setPosition(0.4);
            }
            if (gamepad1.right_trigger > 0.1) {
                robot.wobble.setPower(gamepad1.right_trigger);
//                 runMotor_LiftWobble (0);
            } else if (gamepad1.left_trigger > 0.1) {
//                 runMotor_DropWobble (0);
                robot.wobble.setPower(-gamepad1.left_trigger);
            }else{
                robot.wobble.setPower(0);
            }
            if(gamepad1.dpad_down){
//                robot.wobble2.setPosition(0.87);
                runServo_DropWobble2(0);
            }
            else if(gamepad1.dpad_up){
//                robot.wobble2.setPosition(0);
                runServo_LiftWobble2(0);
            }
            if(gamepad1.dpad_left){
//                robot.grab2.setPosition(0);
                openServo_Grab2(0);
            }
            else if(gamepad1.dpad_right){
//                robot.grab2.setPosition(0.6);
                closeServo_Grab2(0);
            }

//            if (gamepad1.a){
//                strafeRightPID_Encoder(0.5,9.5,1);
//            }
//            if (gamepad1.b){
//                strafeRightPID_Encoder(1,24,5);
//            }
//            //Gamepad2 stuff
//
//            double FlipCollec;
//            double ShooterPwr;
//            if (gamepad2.right_trigger > 0.1) {
//                robot.lift.setPower(-gamepad2.right_trigger);
//                robot.collection.setPower(gamepad2.right_trigger);
//            } else if (gamepad2.left_trigger > 0.1) {
//                robot.lift.setPower(gamepad2.left_trigger);
//                robot.collection.setPower(-gamepad2.left_trigger);
//            } else {
//                robot.lift.setPower(0);
//                robot.collection.setPower(0);
//
//            }
//
//            double drive2 = -gamepad2.right_stick_y;
//            if (gamepad2.left_bumper) {
////                shooterPower    = Range.clip(drive2, -0.7, 0.7) ;
//                shooterPower = Velocity_Powershot;
//            } else if(gamepad2.right_bumper){
//                // shooterPower    = Range.clip(drive2, -0.76, 0.76) ;
////                shooterPower    = Range.clip(drive2, -0.95, 0.95) ;
//                shooterPower = Velocity_HighGoal;
//            } else {
//                shooterPower = 0;
//            }
//            robot.shooter.setVelocity(shooterPower);
//
//            if (gamepad2.dpad_down) {
////                robot.trigger.setPosition(robot.trigger_openPos);
//                runServo_TriggerOpen(0);
//            }else{
////                robot.trigger.setPosition(robot.trigger_closePos);
//                runServo_TriggerClose(0);
//            }
//            if (gamepad2.x) {
//                robot.flip.setPower(1);
//            }else if(gamepad2.y) {
//                robot.flip.setPower(-1);
//            }else{
//                robot.flip.setPower(0);
//
//            }

        }
    }
}