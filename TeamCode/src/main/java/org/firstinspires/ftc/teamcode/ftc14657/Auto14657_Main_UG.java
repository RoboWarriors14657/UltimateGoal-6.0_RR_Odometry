package org.firstinspires.ftc.teamcode.ftc14657;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.acmerobotics.roadrunner.util.Angle;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.advanced.PoseStorage;

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

    Pose2d newLastPose;
    double rotationSpeed = 0;

    public void runOpMode() throws InterruptedException{
        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        init(hardwareMap);

        // reset PoseStorage.currentPose
        PoseStorage.currentPose = new Pose2d();

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

        double collectionSpeed = 0;
//        if(isPowerShotFirst && starterStackNo != 4) {
        if(isPowerShotFirst) {

            rotationSpeed = Velocity_Powershot;
            traj1 = drive.trajectoryBuilder(new Pose2d(), true)
//                    .lineToLinearHeading(new Pose2d(-61, 1, Math.toRadians(0)),
                    .lineToLinearHeading(new Pose2d(-62, 0.5, Math.toRadians(0)),
                            new MinVelocityConstraint(
                                    Arrays.asList(
                                            new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                            new MecanumVelocityConstraint(38, DriveConstants.TRACK_WIDTH)
                                    )
                            ),
                            new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL)
                    )
//                    .addDisplacementMarker(45, () -> {
//                        startShooterThread(0, Velocity_Powershot);
//                    })
                    .build();
//            traj1_2 = drive.trajectoryBuilder(traj1.end(), true)
//                    .lineToLinearHeading(new Pose2d(-61, 9, Math.toRadians(0)),
//                            new MinVelocityConstraint(
//                                    Arrays.asList(
//                                            new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
//                                            new MecanumVelocityConstraint(25, DriveConstants.TRACK_WIDTH)
//                                    )
//                            ),
//                            new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL)
//                    )
//                    .build();
//            traj1_3 = drive.trajectoryBuilder(traj1_2.end(), true)
//                    .lineToLinearHeading(new Pose2d(-61, 17, Math.toRadians(0)),
//                            new MinVelocityConstraint(
//                                    Arrays.asList(
//                                            new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
//                                            new MecanumVelocityConstraint(25, DriveConstants.TRACK_WIDTH)
//                                    )
//                            ),
//                            new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL)
//                    )
//                    .build();

//            rotationSpeed = Velocity_Powershot;
        } else {
            traj1 = drive.trajectoryBuilder(new Pose2d(), true)

//                    .lineToLinearHeading(new Pose2d(-55, 0, Math.toRadians(-23)))
                    .lineToLinearHeading(new Pose2d(-53.5, 0, Math.toRadians(-15)))
                    .build();

//            rotationSpeed = 0.95;
            rotationSpeed = Velocity_HighGoal; //Velocity_HighGoal;

        }
        drive.followTrajectory(traj1);

        //
//        if(isPowerShotFirst &&  starterStackNo != 4) {
        if(isPowerShotFirst ) {

            drive.turn(Angle.normDelta(Math.toRadians(0) - drive.getPoseEstimate().getHeading()));

            if(starterStackNo != 4) {
                startShooterThread(2000, rotationSpeed, false);
            } else {
                startShooterThread(2000, rotationSpeed, true);
            }
            if (isRedAlliance) {
                if(!drive.isBusy()) {
//                    sleep(500);
                    robot.collection.setVelocity(Velocity_Collection_Powershot);
//                    robot.collection.setPower(0.5);
//                    sleep(200);
                    sleep(400);
                    robot.collection.setPower(0);
                    if(starterStackNo != 4) {
                        sleep(1000);
                    } else {
                        sleep(300);
                    }
                }

//                drive.turn(Math.toRadians(-4));
                drive.turn(Math.toRadians(5));
                if(!drive.isBusy()) {
//                    robot.collection.setPower(0.5);
                    robot.collection.setVelocity(Velocity_Collection_Powershot);
//                sleep(700);
                    sleep(700);
                    robot.collection.setPower(0);
                    if(starterStackNo != 4) {
                        sleep(1000);
                    }  else {
                        sleep(300);
                    }
                }

//                drive.followTrajectory(traj1_3);
                drive.turn(Math.toRadians(-4.35));
                if(!drive.isBusy()) {
//                    robot.collection.setPower(0.5);
                    robot.collection.setVelocity(Velocity_Collection_Powershot);
                    sleep(1000);
                    robot.collection.setPower(0);
                }

                traj1 = traj1_3;
//                newLastPose = traj1.end().plus(new Pose2d(-61, 1, Math.toRadians(-10)));



            } else {
            }
            stopShooterThread();
        } else {

//            startShooterThread(2000, rotationSpeed);
            startShooterThread(2000, rotationSpeed, false);
            if (isRedAlliance) {
//                telemetry.addData(String.format("  Voltage: "), batteryVoltageSensor.getVoltage());
//                telemetry.update();
//                startCollectionThread(1500, getCollectionSpeed());
                startCollectionThread(1200, Velocity_Collection);

            } else {
            }

            stopCollectionThread();
            stopShooterThread();

//            newLastPose = traj1.end();
        }

    }


    public void moveToTargetZone (boolean isRedAlliance, boolean isPowerShotFirst){

        drive.update();
        // Read pose
        Pose2d newLastPose = drive.getPoseEstimate();

        if(isRedAlliance) {
            if (starterStackNo == 1) {
                traj2 = drive.trajectoryBuilder(newLastPose, true)
                        .lineToLinearHeading(new Pose2d(-86, 19, Math.toRadians(0)))
                        .addDisplacementMarker(10, () -> {
                            runServo_DropWobble2(0);
//                            runServo_FlipDown(0);
                            runServo_TriggerClose(0);
                        })
                        .build();

            } else if (starterStackNo == 4) {
                traj2 = drive.trajectoryBuilder(newLastPose, true)
//                        .lineToLinearHeading(new Pose2d(-114, 44, Math.toRadians(0)))
                        .lineToLinearHeading(new Pose2d(-105, 38, Math.toRadians(-45)))
                        .addDisplacementMarker(10, () -> {
                            runServo_DropWobble2(0);
//                            runServo_FlipDown(0);
                            runServo_TriggerClose(0);
                        })
                        .build();
            } else {
                traj2 = drive.trajectoryBuilder(newLastPose, true)
                        .lineToLinearHeading(new Pose2d(-63, 39, Math.toRadians(-45)),
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
//        robot.flip.setPower(0);
        openServo_Grab2 (300);
//        sleep(3000);


    }

    public void collectAdditionlRings (boolean foruRings){

        if (starterStackNo == 1) {
            traj3 = drive.trajectoryBuilder(traj2.end(), false)
                    .lineToLinearHeading(new Pose2d(-35, 18.5, Math.toRadians(0)),
                            new MinVelocityConstraint(
                                    Arrays.asList(
                                            new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                            new MecanumVelocityConstraint(38, DriveConstants.TRACK_WIDTH)
                                    )
                            ),
                            new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL)
                    )
                    .build();
            startCollectionThread(200, Velocity_Collection);
            drive.followTrajectory(traj3);
            sleep(1000);
            // stop the ShooterWheelThread.
            stopCollectionThread();
        } else if (starterStackNo == 4) {
            if(foruRings){

                traj3_3 = drive.trajectoryBuilder(traj2.end(), false)
//                        .lineToLinearHeading(new Pose2d(-50, 18, Math.toRadians(-10)),
                        .lineToLinearHeading(new Pose2d(-50, 17.5, Math.toRadians(-5)),
                                new MinVelocityConstraint(
                                        Arrays.asList(
                                                new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
//                                                new MecanumVelocityConstraint(40, DriveConstants.TRACK_WIDTH)
                                                new MecanumVelocityConstraint(41.5, DriveConstants.TRACK_WIDTH)
                                        )
                                ),
                                new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL)
                        )
                        .addDisplacementMarker(10, () -> {
                            startShooterThread(0, Velocity_HighGoal_middle, true);
                            startCollectionThread(0, Velocity_Collection);
                            runServo_TriggerOpen(0);
                         })
                        .build();
                drive.followTrajectory(traj3_3);
                sleep(1000);
                runServo_TriggerClose(300);

                stopShooterThread();

                traj3_4 = drive.trajectoryBuilder(traj3_3.end(), false)
//                        .lineToLinearHeading(new Pose2d(-31, 18, Math.toRadians(-10)),
                        .lineToLinearHeading(new Pose2d(-31, 17.5, Math.toRadians(-5)),
                                new MinVelocityConstraint(
                                        Arrays.asList(
                                                new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                                new MecanumVelocityConstraint(10, DriveConstants.TRACK_WIDTH)
                                        )
                                ),
                                new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL)
                        )
                        .build();

                drive.followTrajectory(traj3_4);

//                stopCollectionThread();

            } else {
                traj3 = drive.trajectoryBuilder(traj2.end(), false)
                        .lineToLinearHeading(new Pose2d(-78, 18, Math.toRadians(0)))
                        .build();

                traj3_3 = drive.trajectoryBuilder(traj3.end(), false)
                        .lineToLinearHeading(new Pose2d(-45, 18, Math.toRadians(0)))
                        .build();

                drive.followTrajectory(traj3);
                drive.followTrajectory(traj3_3);

                traj3_4 = drive.trajectoryBuilder(traj3_3.end(), false)
                        .lineToLinearHeading(new Pose2d(-31, 18, Math.toRadians(0)),
                                new MinVelocityConstraint(
                                        Arrays.asList(
                                                new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                                new MecanumVelocityConstraint(5, DriveConstants.TRACK_WIDTH)
                                        )
                                ),
                                new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL)
                        )
                        .build();
                startCollectionThread(50, Velocity_Collection);
                drive.followTrajectory(traj3_4);
                sleep(1000);
                stopCollectionThread();
            }
        } else {
            // target zone 1 starterStackNo == 0
            traj3 = traj2;
        }

    }

    public void grabSecondWobble (){

        if (starterStackNo == 0){

            traj4 = drive.trajectoryBuilder(traj3.end(), false)
                    .lineToLinearHeading(new Pose2d(-31, 30, Math.toRadians(0)),
                            new MinVelocityConstraint(
                                    Arrays.asList(
                                            new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                            new MecanumVelocityConstraint(25, DriveConstants.TRACK_WIDTH)
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
                    .lineToLinearHeading(new Pose2d(-31.5, 30, Math.toRadians(0)),
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

        runMotor_DropWobble(400);
        closeServo_Grab(400);
        if(starterStackNo == 4){
            stopCollectionThread();
        }


    }


    public void shootAdditionalRings (){
        if (starterStackNo == 0){
            traj5 = drive.trajectoryBuilder(traj4.end(), false)
//                    .lineToLinearHeading(new Pose2d(-60, 0, Math.toRadians(0)))
                    .splineToSplineHeading(new Pose2d(-71, 0, Math.toRadians(90)), Math.toRadians(0),
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
                        runServo_LiftWobble2(0);
                        robot.wobble.setPower(-0.3);
                    })
                    .build();
            drive.followTrajectory(traj5);
        } else {
            traj5 = drive.trajectoryBuilder(traj4.end(), false)
//                    .lineToLinearHeading(new Pose2d(-56, 25, Math.toRadians(-5.5)))
                    .lineToLinearHeading(new Pose2d(-54.75, 25, Math.toRadians(0)))
                    .addDisplacementMarker(3, () -> {
                        closeServo_Grab(0);
                        runServo_TriggerOpen(0);
                        runServo_LiftWobble2(0);
                        robot.wobble.setPower(-0.3);
                    })
                    .build();

            drive.followTrajectory(traj5);

//        // revert the collection
            if (starterStackNo == 4 || starterStackNo == 1) {
                robot.shooter.setVelocity(-200);
                robot.collection.setPower(-0.7);
                robot.lift.setPower(1);
                sleep(200);
                robot.collection.setPower(0);
                robot.lift.setPower(0);

                sleep(150);
            }

//            startShooterThread(2000, Velocity_HighGoal);
            startShooterThread(1500, Velocity_HighGoal, true);
            startCollectionThread(1200, Velocity_Collection);
            stopCollectionThread();
            stopShooterThread();
        }
    }

    public void dropSecondWobble (){
        if (starterStackNo == 0){
            traj6 = drive.trajectoryBuilder(traj5.end(), false)
                    .lineToLinearHeading(new Pose2d(-71, 20, Math.toRadians(98)),
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

            openServo_Grab(400, true);
            runMotor_LiftWobble(700);

            traj7 = drive.trajectoryBuilder(traj6.end(), false)
                    .lineToLinearHeading(new Pose2d(-71, 25, Math.toRadians(98)))
                    .build();
            drive.followTrajectory(traj7);
//            closeServo_Grab(500);
//            runServo_FlipDown(1000);

        } else if(starterStackNo == 1){
            traj6 = drive.trajectoryBuilder(traj5.end(), false)
                    .lineToLinearHeading(new Pose2d(-76, 18, Math.toRadians(185)))
                    .build();
            drive.followTrajectory(traj6);

            openServo_Grab(400, true);
            runMotor_LiftWobble(700);

            traj7 = drive.trajectoryBuilder(traj6.end(), false)
                    .lineToLinearHeading(new Pose2d(-81, 18, Math.toRadians(185)))
                    .addDisplacementMarker(0, () -> {
                        runMotor_LiftWobble(0);
                        runServo_DropWobble2(0);
                    })
                    .build();
            drive.followTrajectory(traj7);
//            closeServo_Grab(500);

        }  else if(starterStackNo == 4) {

            traj6 = drive.trajectoryBuilder(traj5.end(), false)
                    .lineToLinearHeading(new Pose2d(-105, 35, Math.toRadians(-170)))
                    .build();
            drive.followTrajectory(traj6);

            openServo_Grab(400, false);
            runMotor_LiftWobble(200);
            traj7 = drive.trajectoryBuilder(traj6.end(), false)
                    .lineToLinearHeading(new Pose2d(-78, 35, Math.toRadians(-170)),
                            new MinVelocityConstraint(
                                    Arrays.asList(
                                            new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                            new MecanumVelocityConstraint(75, DriveConstants.TRACK_WIDTH)
                                    )
                            ),
                            new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL)
                    )
                    .addDisplacementMarker(0, () -> {
                        runMotor_LiftWobble(0);
                        runServo_DropWobble2(0);
                    })
                    .build();
            drive.followTrajectory(traj7);
//            closeServo_Grab(500);

        }

    }
    public void startShooterThread (long runtimeMiliSec, double rotationSpeed, boolean isFastShoot){
        ShooterWheelThread = new ShooterWheelThread(robot, rotationSpeed);

        if( opModeIsActive()) {
            runtime.reset();
            ShooterWheelThread.start();
            if (!isFastShoot){
                sleep(runtimeMiliSec);
            } else {
                while (runtime.milliseconds() <= runtimeMiliSec) {
                    if (robot.shooter.getVelocity() >= rotationSpeed) {
                        break;
                    }
                }
            }
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

//    public double getCollectionSpeed (){
//        double collectionSpeed = 1;
//        if(batteryVoltageSensor.getVoltage() > fullPowerVoltage)
//        {
//            collectionSpeed = fullPowerVoltage/(batteryVoltageSensor.getVoltage());
//        }
//        return collectionSpeed;
//    }




}

