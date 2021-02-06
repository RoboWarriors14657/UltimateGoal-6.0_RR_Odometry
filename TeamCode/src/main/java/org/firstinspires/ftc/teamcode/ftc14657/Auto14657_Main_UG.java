package org.firstinspires.ftc.teamcode.ftc14657;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;

import java.util.Arrays;

public class Auto14657_Main_UG extends Auto14657_Base_UG {

    Thread  CollectionThread;
    Thread  ShooterWheelThread;

    Trajectory traj1; // from start to shooting
    Trajectory traj1_2; // 2nd power shot
    Trajectory traj1_3; // 3rd power shot
    Trajectory traj2; // from shooting to drop zones
    Trajectory traj3; // from drop zone to additional ring
    Trajectory traj3_3; // from drop zone to before additional ring
    Trajectory traj3_4; // from drop zone to additional ring
    Trajectory traj4; // from ring to the second wobble
    Trajectory traj5; // from the second wobble to the launch zone
    Trajectory traj6; // from the launch zone to the drop zone
    Trajectory traj7; // from the drop zone to park on the launch line

    public void runOpMode() throws InterruptedException{
        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        init(hardwareMap);

        telemetry.addData(">", "Press Play to start");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        runAutoActions();

        if (tfod != null) {
            tfod.shutdown();
        }
        telemetry.addData("Path", "Complete");
        telemetry.update();
    }

    public void runAutoActions(){
    }

    public void moveToLaunchLineShoot (boolean isRedAlliance, boolean isPowerShotFirst){

        telemetry.addData(String.format("  starterStackNo: "), starterStackNo);
        telemetry.update();
        double rotationSpeed = 0;
        if(isPowerShotFirst && starterStackNo != 4) {
            traj1 = drive.trajectoryBuilder(new Pose2d(), true)
                    .lineToLinearHeading(new Pose2d(-61, 0, Math.toRadians(0)))
                    .build();
            traj1_2 = drive.trajectoryBuilder(traj1.end(), true)
                    .lineToLinearHeading(new Pose2d(-61, 8, Math.toRadians(0)),
                            new MinVelocityConstraint(
                                    Arrays.asList(
                                            new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                            new MecanumVelocityConstraint(38, DriveConstants.TRACK_WIDTH)
                                    )
                            ),
                            new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL)
                    )
                    .build();
            traj1_3 = drive.trajectoryBuilder(traj1_2.end(), true)
                    .lineToLinearHeading(new Pose2d(-61, 16, Math.toRadians(0)),
                            new MinVelocityConstraint(
                                    Arrays.asList(
                                            new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                            new MecanumVelocityConstraint(38, DriveConstants.TRACK_WIDTH)
                                    )
                            ),
                            new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL)
                    )
                    .build();
//            rotationSpeed = 0.80;
            rotationSpeed = Velocity_Powershot;
        } else {
            traj1 = drive.trajectoryBuilder(new Pose2d(), true)
                    .lineToLinearHeading(new Pose2d(-58, 0, Math.toRadians(-22)),
                            new MinVelocityConstraint(
                                    Arrays.asList(
                                            new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                            new MecanumVelocityConstraint(45, DriveConstants.TRACK_WIDTH)
                                    )
                            ),
                            new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL)
                    )
                    .build();

//            rotationSpeed = 0.95;
            rotationSpeed = Velocity_HighGoal;
        }
        drive.followTrajectory(traj1);

        //
        if(isPowerShotFirst &&  starterStackNo != 4) {
            startShooterThread(2000, rotationSpeed);
            if (isRedAlliance) {

                robot.collection.setPower(0.5);
                sleep(200);
                robot.collection.setPower(0);
                sleep(200);

                drive.followTrajectory(traj1_2);
                robot.collection.setPower(0.5);
                sleep(550);
                robot.collection.setPower(0);
                sleep(200);

                drive.followTrajectory(traj1_3);
                robot.collection.setPower(0.5);
                sleep(1000);
                robot.collection.setPower(0);

                traj1 = traj1_3;


            } else {
            }
            stopShooterThread();
        } else {
            startShooterThread(2000, rotationSpeed);
            if (isRedAlliance) {

                startCollectionThread(1500, 1);

            } else {
            }
            stopShooterThread();
            stopCollectionThread();
        }

    }


    public void moveToTargetZone (boolean isRedAlliance, boolean isPowerShotFirst){

        if(isRedAlliance) {
            if (starterStackNo == 1) {
                traj2 = drive.trajectoryBuilder(traj1.end(), true)
                        .lineToLinearHeading(new Pose2d(-90, 18, Math.toRadians(0)))
                        .addDisplacementMarker(10, () -> {
                            runServo_DropWobble2(0);
                            runServo_FlipDown(0);
                            runServo_TriggerClose(0);
                        })
                        .build();

            } else if (starterStackNo == 4) {
                traj2 = drive.trajectoryBuilder(traj1.end(), true)
                        .lineToLinearHeading(new Pose2d(-114, 41, Math.toRadians(0)))
                        .addDisplacementMarker(10, () -> {
                            runServo_DropWobble2(0);
                            runServo_FlipDown(0);
                            runServo_TriggerClose(0);
                        })
                        .build();
            } else {
                traj2 = drive.trajectoryBuilder(traj1.end(), true)
                        .lineToLinearHeading(new Pose2d(-68, 41, Math.toRadians(0)),
                                new MinVelocityConstraint(
                                        Arrays.asList(
                                                new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                                new MecanumVelocityConstraint(38, DriveConstants.TRACK_WIDTH)
                                        )
                                ),
                                new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL)
                        )
                        .addDisplacementMarker(10, () -> {
                            runServo_DropWobble2(0);
                            runServo_TriggerClose(0);
                        })
                        .build();
            }
        }



        drive.followTrajectory(traj2);
        robot.flip.setPower(0);
        openServo_Grab2 (300);
//        sleep(3000);


    }

    public void collectAdditionlRings (){

        if (starterStackNo == 1) {
            traj3 = drive.trajectoryBuilder(traj2.end(), false)
                    .lineToLinearHeading(new Pose2d(-47, 15, Math.toRadians(0)),
                            new MinVelocityConstraint(
                                    Arrays.asList(
                                            new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                            new MecanumVelocityConstraint(38, DriveConstants.TRACK_WIDTH)
                                    )
                            ),
                            new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL)
                    )
                    .build();
            startCollectionThread(200, 1);
            drive.followTrajectory(traj3);
            sleep(1000);
            // stop the ShooterWheelThread.
            stopCollectionThread();
        } else if (starterStackNo == 4) {

            traj3 = drive.trajectoryBuilder(traj2.end(), false)
                    .lineToLinearHeading(new Pose2d(-78, 13, Math.toRadians(0)),
                            new MinVelocityConstraint(
                                    Arrays.asList(
                                            new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                            new MecanumVelocityConstraint(45, DriveConstants.TRACK_WIDTH)
                                    )
                            ),
                            new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL)
                    )
                    .build();
            traj3_3 = drive.trajectoryBuilder(traj3.end(), false)
                    .lineToLinearHeading(new Pose2d(-48, 13, Math.toRadians(0)),
                            new MinVelocityConstraint(
                                    Arrays.asList(
                                            new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                            new MecanumVelocityConstraint(40, DriveConstants.TRACK_WIDTH)
                                    )
                            ),
                            new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL)
                    )
                    .build();

            drive.followTrajectory(traj3);
            drive.followTrajectory(traj3_3);

            traj3_4 = drive.trajectoryBuilder(traj3_3.end(), false)
                    .lineToLinearHeading(new Pose2d(-35, 13, Math.toRadians(0)),
                        new MinVelocityConstraint(
                        Arrays.asList(
                                new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                new MecanumVelocityConstraint(5, DriveConstants.TRACK_WIDTH)
                        )
                        ),
                        new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL)
                    )
                    .build();
            startCollectionThread(50, 1);
            drive.followTrajectory(traj3_4);
            sleep(200);
            stopCollectionThread();
        } else {
            // target zone 1 starterStackNo == 0
            traj3 = traj2;
        }

    }

    public void collectFourthlRings (){

       if (starterStackNo == 4) {
            traj3_4 = drive.trajectoryBuilder(traj3_3.end(), false)
                    .lineToLinearHeading(new Pose2d(-37.5, 15.5, Math.toRadians(0)),
                            new MinVelocityConstraint(
                                    Arrays.asList(
                                            new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                            new MecanumVelocityConstraint(70, DriveConstants.TRACK_WIDTH)
                                    )
                            ),
                            new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL)
                    )
                    .build();
            startCollectionThread(50, 1);
            drive.followTrajectory(traj3_4);
            sleep(200);
            stopCollectionThread();
        }

    }

    public void grabSecondWobble (){

        if (starterStackNo == 0){

            traj4 = drive.trajectoryBuilder(traj3.end(), false)
                    .lineToLinearHeading(new Pose2d(-37, 30, Math.toRadians(0)),
                            new MinVelocityConstraint(
                                    Arrays.asList(
                                            new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                            new MecanumVelocityConstraint(38, DriveConstants.TRACK_WIDTH)
                                    )
                            ),
                            new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL)
                    )
                    .addDisplacementMarker(3, () -> {
                        openServo_Grab(0, true);
                    })
                    .build();
        } else if(starterStackNo == 1){
            traj4 = drive.trajectoryBuilder(traj3.end(), false)
                    .lineToLinearHeading(new Pose2d(-30, 30, Math.toRadians(0)),
                            new MinVelocityConstraint(
                                    Arrays.asList(
                                            new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                            new MecanumVelocityConstraint(25, DriveConstants.TRACK_WIDTH)
                                    )
                            ),
                            new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL)
                    )
                    .addDisplacementMarker(3, () -> {
                        openServo_Grab(0, false);
//                        runServo_FlipUp(0);
                    })
                    .build();

        } else if(starterStackNo == 4){
            traj4 = drive.trajectoryBuilder(traj3_4.end(), false)
                    .lineToLinearHeading(new Pose2d(-28.5, 30, Math.toRadians(0)),
                            new MinVelocityConstraint(
                                    Arrays.asList(
                                            new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                            new MecanumVelocityConstraint(25, DriveConstants.TRACK_WIDTH)
                                    )
                            ),
                            new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL)
                    )
//                    .lineToLinearHeading(new Pose2d(-30, 30, Math.toRadians(0)))
                    .addDisplacementMarker(3, () -> {

                        openServo_Grab(0, false);
//                        runServo_FlipUp(0);
                    })
                    .build();

        }
//        if (starterStackNo != 0) {
//            runServo_FlipUp(300);
//        }
        drive.followTrajectory(traj4);
        runMotor_DropWobble(500);
//        closeServo_Grab(400);
        closeServo_Grab(200);

    }

    public void shootRings (){

        if (starterStackNo == 4) {
            traj5 = drive.trajectoryBuilder(traj3_4.end(), false)
                    //                    .lineToLinearHeading(new Pose2d(-60, 29, Math.toRadians(0)))
                    .lineToLinearHeading(new Pose2d(-61.5, 20, Math.toRadians(-8.5)))
                    .addDisplacementMarker(3, () -> {
                        closeServo_Grab(0);
                        runServo_TriggerOpen(0);
                    })
                    .build();

            drive.followTrajectory(traj5);

            //        // revert the collection

            robot.shooter.setVelocity(-100);
            robot.collection.setPower(-0.5);
            //                sleep(150);
            sleep(200);
            robot.collection.setPower(0);
            //                sleep(200);
            sleep(150);

            //        startShooterThread(2000, 0.95);
            //        startCollectionThread(1500, 1);
            startShooterThread(2000, Velocity_HighGoal);
            startCollectionThread(1000, 1);
            stopShooterThread();
            stopCollectionThread();
            traj3_3 = traj5;
        }

    }

    public void shootAdditionalRings (){
        if (starterStackNo == 0){
            traj5 = drive.trajectoryBuilder(traj4.end(), false)
//                    .lineToLinearHeading(new Pose2d(-60, 0, Math.toRadians(0)))
                    .splineToSplineHeading(new Pose2d(-82, 0, Math.toRadians(90)), Math.toRadians(0),
                            new MinVelocityConstraint(
                                    Arrays.asList(
                                            new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                            new MecanumVelocityConstraint(38, DriveConstants.TRACK_WIDTH)
                                    )
                            ),
                            new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL)
                    )
                    .addDisplacementMarker(3, () -> {
                        closeServo_Grab(0);
                        runServo_TriggerOpen(0);
                    })
                    .build();
            drive.followTrajectory(traj5);
        } else {
            traj5 = drive.trajectoryBuilder(traj4.end(), false)
//                    .lineToLinearHeading(new Pose2d(-60, 29, Math.toRadians(0)))
                    .lineToLinearHeading(new Pose2d(-61.5, 20, Math.toRadians(-7.5)))
                    .addDisplacementMarker(3, () -> {
                        closeServo_Grab(0);
                        runServo_TriggerOpen(0);
                    })
                    .build();

            drive.followTrajectory(traj5);

//        // revert the collection
            if (starterStackNo == 4 || starterStackNo == 1) {
                robot.shooter.setVelocity(-100);
                robot.collection.setPower(-0.5);
//                sleep(150);
                sleep(200);
                robot.collection.setPower(0);
//                sleep(200);
                sleep(150);
            }
//        startShooterThread(2000, 0.95);
//        startCollectionThread(1500, 1);
            startShooterThread(2000, Velocity_HighGoal);
            startCollectionThread(1000, 1);
            stopShooterThread();
            stopCollectionThread();
        }
    }

    public void dropSecondWobble (){
        if (starterStackNo == 0){
            traj6 = drive.trajectoryBuilder(traj5.end(), false)
                    .lineToLinearHeading(new Pose2d(-82, 20, Math.toRadians(90)),
                            new MinVelocityConstraint(
                                    Arrays.asList(
                                            new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                            new MecanumVelocityConstraint(38, DriveConstants.TRACK_WIDTH)
                                    )
                            ),
                            new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL)
                    )
                    .build();
            drive.followTrajectory(traj6);

//            openServo_Grab(400, true);
//            runMotor_LiftWobble(700);
//
//            traj7 = drive.trajectoryBuilder(traj6.end(), false)
//                    .lineToLinearHeading(new Pose2d(-82, 0, Math.toRadians(180)))
//                    .build();
//            drive.followTrajectory(traj7);
//            closeServo_Grab(500);
//            runServo_FlipDown(1000);

        } else if(starterStackNo == 1){
            traj6 = drive.trajectoryBuilder(traj5.end(), false)
                    .lineToLinearHeading(new Pose2d(-83, 15, Math.toRadians(180)))
                    .build();
            drive.followTrajectory(traj6);

//            openServo_Grab(400, true);
//            runMotor_LiftWobble(700);

//            traj7 = drive.trajectoryBuilder(traj6.end(), false)
//                    .lineToLinearHeading(new Pose2d(-78, 0, Math.toRadians(180)))
//                    .build();
//            drive.followTrajectory(traj7);
//            closeServo_Grab(500);

        }  else if(starterStackNo == 4) {

            traj6 = drive.trajectoryBuilder(traj5.end(), false)
                    .lineToLinearHeading(new Pose2d(-118, 31, Math.toRadians(-180)))
                    .build();
            drive.followTrajectory(traj6);

            openServo_Grab(400, false);
            runMotor_LiftWobble(200);
            traj7 = drive.trajectoryBuilder(traj6.end(), false)
                    .lineToLinearHeading(new Pose2d(-86, 31, Math.toRadians(-180)))
                    .addDisplacementMarker(0, () -> {
                        runMotor_LiftWobble(0);
                    })
                    .build();
            drive.followTrajectory(traj7);
            closeServo_Grab(500);

        }

    }
    public void startShooterThread (long runtimeMiliSec, double rotationSpeed){
        ShooterWheelThread = new ShooterWheelThread(robot, rotationSpeed);

        if( opModeIsActive()) {
            ShooterWheelThread.start();
            sleep(runtimeMiliSec);
            idle();
        }
    }

    public void stopShooterThread (){
        if( opModeIsActive()) {
            ShooterWheelThread.interrupt();
            robot.shooter.setPower(0);
        }
    }

    public void startCollectionThread (long runtimeMiliSec, double rotationSpeed){
        CollectionThread = new CollectionThread(robot, rotationSpeed);

        if( opModeIsActive()) {
            CollectionThread.start();
            sleep(runtimeMiliSec);
            idle();
        }
    }

    public void stopCollectionThread (){
        if( opModeIsActive()) {
            CollectionThread.interrupt();
            robot.collection.setPower(0);
            robot.lift.setPower(0);
        }
    }




}

