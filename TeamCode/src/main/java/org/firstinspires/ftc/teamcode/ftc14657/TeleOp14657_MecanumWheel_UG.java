package org.firstinspires.ftc.teamcode.ftc14657;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.util.Angle;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.advanced.PoseStorage;

@TeleOp(name="TeleOp UG OnBot", group="Linear Opmode")
public class TeleOp14657_MecanumWheel_UG extends  Auto14657_Base_UG {

    private ElapsedTime runtime = new ElapsedTime();
    double shooterPower;

    Trajectory traj1;
    Trajectory traj2;
    boolean was_dpad_left = false;
    boolean was_dpad_right = false;
    double drive_power = 0.85;
    double turn_power = 0.5;
    double slowDrive_power = 0.3;
    double slowTurn_power = 0.2;

    int count_dpad_left = 0;

    // The coordinates we want the bot to automatically go to when we press the A button
    Pose2d targetAPose = new Pose2d(-55, 35, Math.toRadians(0));

    // The location we want the bot to automatically go to when we press the B button
    Pose2d targetBPose = new Pose2d(-58, 0, Math.toRadians(0));

    // The angle we want to align
    double targetAngle = Math.toRadians(5);


    enum Mode_driver2 {
        DRIVER_MODE,
        SHOOTING_MODE,
        POWERSHOT_MODE,
        AUTOMATIC_CONTROL
    }

    enum Mode_driver1 {
        DRIVER_MODE_1,
        SLOW_MODE_1,
        AUTOMATIC_CONTROL_1
    }

    public void init(HardwareMap ahwMap) {
//        robot.init(hardwareMap);
        robot.init(hardwareMap);
        drive = new SampleMecanumDrive(hardwareMap);
    }

    @Override
    public void runOpMode() throws InterruptedException{
        // set default mode
        Mode_driver2 mode = Mode_driver2.DRIVER_MODE;
        Mode_driver1 mode_1 = Mode_driver1.DRIVER_MODE_1;

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        init(hardwareMap);

        // Retrieve our pose from the PoseStorage.currentPose static field
        // See AutoTransferPose.java for further details
        drive.setPoseEstimate(PoseStorage.currentPose);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();
        robot.shooter.setDirection(DcMotor.Direction.FORWARD);
        // run until the end of the match (driver presses STOP)

        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {
            // Update the drive class
            drive.update();

            // Read pose
            Pose2d poseEstimate = drive.getPoseEstimate();
//        while (opModeIsActive()) {

            telemetry.addData("driver1 mode", mode_1);
            telemetry.addData("driver2 mode", mode);

            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.addData("Velocity", robot.shooter.getVelocity());
            telemetry.update();

            // Driver 2
            switch (mode)
            {
                case DRIVER_MODE:
                    drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                    if (gamepad2.a) {
                        mode = Mode_driver2.SHOOTING_MODE;
                        drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        runtime.reset();
                    } else if (gamepad2.b) {
                        mode = Mode_driver2.POWERSHOT_MODE;
                        drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        runtime.reset();
                    } else if(gamepad2.y){
                        // reset PoseStorage.currentPose
                        PoseStorage.currentPose = new Pose2d();
                        drive.setPoseEstimate(PoseStorage.currentPose);
                        targetAPose = new Pose2d(0, -10.3, Math.toRadians(0));
//                        targetBPose = new Pose2d(-7.28, -42.5, Math.toRadians(0));
                        targetBPose = new Pose2d(-4, -5, Math.toRadians(15));
                    }

                    if (gamepad2.right_trigger > 0.1) {
                        robot.lift.setPower(-gamepad2.right_trigger);
                        robot.collection.setPower(gamepad2.right_trigger);
                    } else if (gamepad2.left_trigger > 0.1) {
                        robot.lift.setPower(gamepad2.left_trigger);
                        robot.collection.setPower(-gamepad2.left_trigger/3);
                    } else {
                        robot.lift.setPower(0);
                        robot.collection.setPower(0);
                    }
                    double drive2 = -gamepad2.right_stick_y;
                    if (gamepad2.left_bumper) {
                        shooterPower = Velocity_Powershot;
                    } else if(gamepad2.right_bumper){
                        shooterPower = Velocity_HighGoal;
                    } else {
                        shooterPower = 0;
                    }
                    robot.shooter.setVelocity(shooterPower);

                    if (gamepad2.dpad_down) {
                        runServo_TriggerOpen(0);
                    }else{
                        runServo_TriggerClose(0);
                    }


                    break;
                case SHOOTING_MODE:
                    if (gamepad2.x)
                    {
                        mode = Mode_driver2.DRIVER_MODE;
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
                            robot.shooter.setVelocity(-200);
                            robot.collection.setPower(-0.7);
                            robot.lift.setPower(1);
//                            robot.collection.setPower(-0.5);
//                            robot.shooter.setVelocity(-100);
                            sleep(200);
                        }
                        else if (runtime.milliseconds() > milisec_Revert && runtime.milliseconds() <= milesec_MotorRotate )
                        {
                            robot.collection.setPower(0);
                            robot.lift.setPower(0);
                            // turn on shooter high speed motor
                            robot.shooter.setVelocity(Velocity_HighGoal);
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

                        mode = Mode_driver2.DRIVER_MODE;
                        drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    }
                    break;
                case POWERSHOT_MODE:
                    if (gamepad2.x)
                    {
                        mode = Mode_driver2.DRIVER_MODE;
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
//                            robot.collection.setPower(-0.5);
//                            robot.shooter.setVelocity(-100);
//                            sleep(150);
                            robot.shooter.setVelocity(-200);
                            robot.collection.setPower(-0.7);
                            robot.lift.setPower(1);
                            sleep(200);
                        }
                        else if (runtime.milliseconds() > milisec_Revert && runtime.milliseconds() <= milesec_MotorRotate )
                        {
                            robot.collection.setPower(0);
                            robot.lift.setPower(0);
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

                            //strafeLeft(7)
                            if (gamepad2.dpad_left)
                            {
                                Trajectory traj1 = drive.trajectoryBuilder(poseEstimate)
                                        .strafeLeft(7)
                                        .build();

                                drive.followTrajectory(traj1);
                                mode = Mode_driver2.AUTOMATIC_CONTROL;
                            }
                            //strafeRight(7)
                            else if (gamepad2.dpad_right)
                            {
                                Trajectory traj2 = drive.trajectoryBuilder(poseEstimate)
                                        .strafeRight(7)
                                        .build();

                                drive.followTrajectory(traj2);
                                mode = Mode_driver2.AUTOMATIC_CONTROL;
                            }

                            // auto power shot
                            if (gamepad2.left_bumper )
                            {
                                Trajectory traj_a;
                                if(count_dpad_left == 0) {
                                    runServo_TriggerOpen(0);
                                    sleep(300);

                                    traj_a = drive.trajectoryBuilder(poseEstimate)
                                            .lineToLinearHeading(targetBPose)
                                            .build();
                                    drive.followTrajectory(traj_a);

                                    robot.collection.setPower(0.5);
                                    robot.lift.setPower(-0.5);
                                    sleep(200);
                                    robot.lift.setPower(0);
                                    robot.collection.setPower(0);

                                    count_dpad_left++;
                                }
                                else if(count_dpad_left == 1) {
                                    traj_a = drive.trajectoryBuilder(poseEstimate)
                                            .lineToLinearHeading(new Pose2d(-4, -12, Math.toRadians(15)))
                                            .build();
                                    drive.followTrajectory(traj_a);

                                    robot.collection.setPower(0.5);
                                    robot.lift.setPower(-0.5);
                                    sleep(250);
                                    robot.lift.setPower(0);
                                    robot.collection.setPower(0);
                                    sleep(200);
                                    count_dpad_left++;
                                }
                                else if(count_dpad_left == 2) {
                                    traj_a = drive.trajectoryBuilder(poseEstimate)
                                            .lineToLinearHeading(new Pose2d(-4, -19, Math.toRadians(15)))
                                            .build();
                                    drive.followTrajectory(traj_a);
                                    robot.collection.setPower(0.5);
                                    robot.lift.setPower(-0.5);
                                    sleep(1000);
                                    robot.lift.setPower(0);
                                    robot.collection.setPower(0);
                                    count_dpad_left++;
                                }

//                                targetBPose = new Pose2d(-4, -5, Math.toRadians(15));
                                if(count_dpad_left == 3) {
                                    mode = Mode_driver2.AUTOMATIC_CONTROL;
                                }
                            }

                        }

                    } else {
                        robot.shooter.setVelocity(0);
                        robot.collection.setPower(0);
                        robot.lift.setPower(0);
                        runServo_TriggerClose(0);
                        // reset runtime
                        runtime.reset();
                        count_dpad_left = 0;
                        mode = Mode_driver2.DRIVER_MODE;
                        drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    }
                    break;

                case AUTOMATIC_CONTROL:
                    // If x is pressed, we break out of the automatic following
//                    if (gamepad2.x) {
////                        drive.cancelFollowing();
//                        mode = Mode_driver2.DRIVER_MODE;
//                    }

                    // If drive finishes its task, cede control to the driver
                    if (!drive.isBusy()) {
                        mode = Mode_driver2.POWERSHOT_MODE;
                    }
                    break;


            }

            // Driver 1 mode

            switch (mode_1)
            {
                case DRIVER_MODE_1:
                    drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                    drive.setWeightedDrivePower(
                            new Pose2d(
                                    -gamepad1.left_stick_y * drive_power,
                                    -gamepad1.left_stick_x * drive_power,
                                    -gamepad1.right_stick_x * turn_power
                            )
                    );

//                    drive.update();

                    if (gamepad1.a) {
                        // If the A button is pressed on gamepad1, we generate a splineTo()
                        // trajectory on the fly and follow it
                        // We switch the state to AUTOMATIC_CONTROL

                        Trajectory traj1 = drive.trajectoryBuilder(poseEstimate)
                                .lineToLinearHeading(targetAPose)
                                .build();

                        drive.followTrajectoryAsync(traj1);

                        mode_1 = Mode_driver1.AUTOMATIC_CONTROL_1;
                    } else if (gamepad1.b) {
                        // If the B button is pressed on gamepad1, we generate a lineTo()
                        // trajectory on the fly and follow it
                        // We switch the state to AUTOMATIC_CONTROL

                        Trajectory traj1 = drive.trajectoryBuilder(poseEstimate)
                                .lineToLinearHeading(targetBPose)
                                .build();

                        drive.followTrajectoryAsync(traj1);

                        mode_1 = Mode_driver1.AUTOMATIC_CONTROL_1;
                    } else if (gamepad1.y) {
                            mode_1 = Mode_driver1.SLOW_MODE_1;
                            runtime.reset();
//                        // If Y is pressed, we turn the bot to the specified angle to reach
//                        // targetAngle (by default, 45 degrees)
//
//                        drive.turnAsync(Angle.normDelta(targetAngle - poseEstimate.getHeading()));
//
//                        mode_1 = Mode_driver1.AUTOMATIC_CONTROL;
                    }

                    break;
                case SLOW_MODE_1:
                    if (gamepad1.x)
                    {
                        mode_1 = Mode_driver1.DRIVER_MODE_1;
                        drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    }

                    drive.setWeightedDrivePower(
                            new Pose2d(
                                    -gamepad1.left_stick_y * slowDrive_power,
                                    -gamepad1.left_stick_x * slowDrive_power,
                                    -gamepad1.right_stick_x * slowTurn_power
                            )
                    );

//                    drive.update();
                    break;
                case AUTOMATIC_CONTROL_1:
                    // If x is pressed, we break out of the automatic following
                    if (gamepad1.x) {
//                        drive.cancelFollowing();
                        mode_1 = Mode_driver1.DRIVER_MODE_1;
                    }

                    // If drive finishes its task, cede control to the driver
                    if (!drive.isBusy()) {
                        mode_1 = Mode_driver1.DRIVER_MODE_1;
                    }
                    break;
            }

            if (gamepad1.left_bumper) {
                openServo_Grab(0, false);
            } else if (gamepad1.right_bumper) {
                closeServo_Grab(0);
            }
            if (gamepad1.right_trigger > 0.1) {
                robot.wobble.setPower(gamepad1.right_trigger);
            } else if (gamepad1.left_trigger > 0.1) {
                robot.wobble.setPower(-gamepad1.left_trigger);
            }else{
                robot.wobble.setPower(0);
            }
            if(gamepad1.dpad_down){
                runServo_DropWobble2(0);
            }
            else if(gamepad1.dpad_up){
                runServo_LiftWobble2(0);
            }
            if(gamepad1.dpad_left){
                openServo_Grab2(0);
            }
            else if(gamepad1.dpad_right){
                closeServo_Grab2(0);
            }

//            if (gamepad1.a){
//                strafeRightPID_Encoder(0.5,9.5,1);
//            }
//            if (gamepad1.b){
//                strafeRightPID_Encoder(1,24,5);
//            }

        }
    }
}