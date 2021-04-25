package org.firstinspires.ftc.teamcode.ftc14657;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.acmerobotics.roadrunner.util.Angle;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.advanced.PoseStorage;

import java.util.Arrays;

@TeleOp(name="TeleOp UG OnBot", group="Linear Opmode")
public class TeleOp14657_MecanumWheel_UG extends  Auto14657_Base_UG {

    private ElapsedTime runtime = new ElapsedTime();
    double shooterPower;

    Trajectory traj1;
    Trajectory traj2;
    boolean was_dpad_left = false;
    boolean was_dpad_right = false;
//    double drive_power = 0.85;
//    double turn_power = 0.5;
    double drive_power = 0.95;
    double turn_power = 0.8;
    double slowDrive_power = 0.3;
    double slowTurn_power = 0.2;

    int count_dpad_left = 0;
    boolean Y_Click = false;

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
                        // if align with the side wall closer to the driver
//                        targetAPose = new Pose2d(0, -11, Math.toRadians(0));
//                        targetBPose = new Pose2d(-4, -5, Math.toRadians(16));
                        //if align with the starting line
//                        targetBPose = new Pose2d(-61, 1, Math.toRadians(0));
                        targetBPose = new Pose2d(-61.5, 0.5, Math.toRadians(-1));
                        robot.setVelocity(robot.shooter, 0);
                        Y_Click = true;
                    }

                    if (gamepad2.right_trigger > 0.1) {
                        robot.lift.setPower(-gamepad2.right_trigger);
                        robot.collection.setPower(gamepad2.right_trigger);
                    } else if (gamepad2.left_trigger > 0.1) {
                        robot.lift.setPower(gamepad2.left_trigger);
                        robot.collection.setPower(-gamepad2.left_trigger);
//                        robot.collection.setPower(-gamepad2.left_trigger/3);
                    } else {
                        robot.lift.setPower(0);
                        robot.collection.setPower(0);
                    }
//                    double drive2 = -gamepad2.right_stick_y;
//                    if (gamepad2.left_bumper) {
//                        shooterPower = Velocity_Powershot;
//                    } else if(gamepad2.right_bumper){
//                        shooterPower = Velocity_HighGoal;
//                    } else {
//                        shooterPower = 0;
//                    }
//                    robot.shooter.setVelocity(shooterPower);

                    if (gamepad2.dpad_down) {
                        runServo_TriggerOpen(0);
                    }else{
                        runServo_TriggerClose(0);
                    }

                    // wobble goal control
                    if (gamepad2.left_bumper) {
                        openServo_Grab(0, false);
                    } else if (gamepad2.right_bumper) {
                        closeServo_Grab(0);
                    }


                    if( !Y_Click ){
//                        robot.setVelocity(robot.shooter, Velocity_HighGoal);
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
                        runServo_LiftWobble2(0);
                        int milisec_Revert = 300;//400;//200;
                        int milesec_MotorRotate = 2000 + milisec_Revert;
//                        int milesec_MotorRotate = 0 + milisec_Revert;
//                        int milisec_Shooter = 1200 + milesec_MotorRotate;

                        // revert the collection
                        if (runtime.milliseconds() <= milisec_Revert)
                        {
                            // move head first
//                            robot.collection.setPower(1);
//                            robot.lift.setPower(-1);
//                            sleep(200);
//                            robot.collection.setPower(0);
//                            robot.lift.setPower(0);
                            // do revert
//                            robot.shooter.setVelocity(-200);
//                            robot.collection.setPower(-0.5);
                            robot.collection.setVelocity(-1000);
                            robot.lift.setPower(0.7);
                            sleep(100);
                        }
//                        else if (runtime.milliseconds() > milisec_Revert && runtime.milliseconds() <= milesec_MotorRotate )
                        else if (runtime.milliseconds() > milisec_Revert && (runtime.milliseconds() <= milesec_MotorRotate && robot.shooter.getVelocity() <= Velocity_HighGoal) )
                        {
                            robot.collection.setPower(0);
                            robot.lift.setPower(0);
                            // turn on shooter high speed motor
//                            robot.shooter.setVelocity(Velocity_HighGoal);
                            robot.setVelocity(robot.shooter, Velocity_HighGoal);

                        }
//                        else if (runtime.milliseconds() > milesec_MotorRotate )
                        else
                        {
                            // open the trigger, once it is open start shooting
                            if (gamepad2.right_trigger > 0.1) {
                                runServo_TriggerOpen(0);
                                sleep(300);

//                                robot.collection.setPower(1);
                                robot.collection.setVelocity(Velocity_Collection);
                                robot.lift.setPower(-1);

                                // reset PoseStorage.currentPose
//                                PoseStorage.currentPose = new Pose2d(0, -11, Math.toRadians(0));;
//                                drive.setPoseEstimate(PoseStorage.currentPose);
//                                targetAPose = new Pose2d(0, -11, Math.toRadians(0));
                            } else {
                                robot.collection.setPower(0);
                                robot.lift.setPower(0);
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
//                    if (gamepad2.b)
                    if (gamepad2.b && !drive.isBusy())
                    {
                        runServo_LiftWobble2(0);
                        int milisec_Revert = 300; //400;//200;
                        int milesec_MotorRotate = 2000 + milisec_Revert;
    //                        int milisec_Shooter = 1200 + milesec_MotorRotate;

                        // revert the collection
                        if (runtime.milliseconds() <= milisec_Revert)
                        {
                            // move head first
//                            robot.collection.setPower(1);
//                            robot.lift.setPower(-1);
//                            sleep(200);
//                            robot.collection.setPower(0);
//                            robot.lift.setPower(0);
                            // do revert
                            robot.shooter.setVelocity(-200);
//                            robot.collection.setPower(-0.5);
                            robot.collection.setVelocity(-1000);
                            robot.lift.setPower(0.7);
                            sleep(100);
                        }
                        else if (runtime.milliseconds() > milisec_Revert && runtime.milliseconds() <= milesec_MotorRotate )
//                        else if (runtime.milliseconds() > milisec_Revert && (runtime.milliseconds() <= milesec_MotorRotate && robot.shooter.getVelocity() <= Velocity_Powershot) )
                        {
                            robot.collection.setPower(0);
                            robot.lift.setPower(0);
                            // turn on shooter high speed motor
//                            robot.shooter.setVelocity(Velocity_Powershot);
                            robot.setVelocity(robot.shooter, Velocity_Powershot);
                        }
//                        else if (runtime.milliseconds() > milesec_MotorRotate ) {
                        else {
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

//                                    traj_a = drive.trajectoryBuilder(poseEstimate)
//                                            .lineToLinearHeading(targetBPose)
//                                            .build();
//                                    drive.followTrajectory(traj_a);

//                                    robot.collection.setPower(0.5);
                                    drive.turn(Angle.normDelta(Math.toRadians(0) - drive.getPoseEstimate().getHeading()));

                                    robot.collection.setVelocity(Velocity_Collection_Powershot);
                                    robot.lift.setPower(-0.5);
                                    sleep(400);
                                    robot.lift.setPower(0);
                                    robot.collection.setPower(0);
                                    sleep(300);

                                    count_dpad_left++;

                                    mode = Mode_driver2.AUTOMATIC_CONTROL;
                                }
                                else if(count_dpad_left == 1) {
//                                    traj_a = drive.trajectoryBuilder(poseEstimate)
//                                            .lineToLinearHeading(new Pose2d(-4, -11, Math.toRadians(18)))
//                                            .build();
//                                    drive.followTrajectory(traj_a);
//                                    drive.turn(Math.toRadians(-9));
                                    drive.turn(Math.toRadians(9));
//                                    robot.collection.setPower(0.5);
                                    robot.collection.setVelocity(Velocity_Collection_Powershot);
                                    robot.lift.setPower(-0.5);
                                    sleep(700);
                                    robot.lift.setPower(0);
                                    robot.collection.setPower(0);
                                    sleep(300);
                                    count_dpad_left++;

                                    mode = Mode_driver2.AUTOMATIC_CONTROL;
                                }
                                else if(count_dpad_left == 2) {
//                                    drive.turn(Math.toRadians(-9));
                                    drive.turn(Math.toRadians(-12));
                                    robot.collection.setVelocity(Velocity_Collection_Powershot);
                                    robot.lift.setPower(-0.5);
                                    sleep(1000);
                                    robot.lift.setPower(0);
                                    robot.collection.setPower(0);
                                    count_dpad_left++;

                                    mode = Mode_driver2.AUTOMATIC_CONTROL;
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

//                        Trajectory traj1 = drive.trajectoryBuilder(poseEstimate)
//                                .lineToLinearHeading(targetAPose)
//                                .build();
//
//                        drive.followTrajectoryAsync(traj1);
//
//                        mode_1 = Mode_driver1.AUTOMATIC_CONTROL_1;
                    } else if (gamepad1.b) {
                        // If the B button is pressed on gamepad1, we generate a lineTo()
                        // trajectory on the fly and follow it
                        // We switch the state to AUTOMATIC_CONTROL

//                        Trajectory traj1 = drive.trajectoryBuilder(poseEstimate)
//                                .lineToLinearHeading(targetBPose)
//                                .build();
//
//                        drive.followTrajectoryAsync(traj1);
//
//                        mode_1 = Mode_driver1.AUTOMATIC_CONTROL_1;


//                        if (poseEstimate.getX() == 0 && poseEstimate.getY() == 0 && poseEstimate.getHeading() == 0) {
                        if(Y_Click) {
                            Trajectory traj1 = drive.trajectoryBuilder(poseEstimate)
                                    .lineToLinearHeading(targetBPose, new MinVelocityConstraint(
                                                    Arrays.asList(
                                                            new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                                            new MecanumVelocityConstraint(38, DriveConstants.TRACK_WIDTH)
                                                    )
                                            ),
                                            new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL)
                                    )
                                    .build();

                            drive.followTrajectoryAsync(traj1);
//                            Y_Click = false;
                            mode_1 = Mode_driver1.AUTOMATIC_CONTROL_1;
                        }
                    } else if (gamepad1.y) {
                            mode_1 = Mode_driver1.SLOW_MODE_1;
                            runtime.reset();
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
//                robot.wobble.setPower(gamepad1.right_trigger);
                //open wings
                robot.leftWing.setPosition(0);
                robot.rightWing.setPosition(0.4);
            } else if (gamepad1.left_trigger > 0.1) {
//                robot.wobble.setPower(-gamepad1.left_trigger);
//                robot.leftWing.setPosition(0.4);
//                robot.rightWing.setPosition(0);
            } else {
                //close wings
                robot.leftWing.setPosition(0.4);
                robot.rightWing.setPosition(0);
                // drive2 also control wobble
                double wobbleArmPower = -gamepad2.right_stick_y;
                if (Math.abs(wobbleArmPower) > 0.001) {
                    robot.wobble.setPower(wobbleArmPower);
                } else {
                    robot.wobble.setPower(0);
                }
//                robot.wobble.setPower(0);
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