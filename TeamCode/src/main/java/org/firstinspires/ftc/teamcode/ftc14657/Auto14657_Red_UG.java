package org.firstinspires.ftc.teamcode.ftc14657;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Red-test rotation", group="OpMode")
public class Auto14657_Red_UG extends Auto14657_Main_UG {
    public void runAutoActions(){


        moveToLaunchLineShoot (true, true);
        moveToTargetZone(true,false);
        collectAdditionlRings ();
////        shootRings();
////        collectFourthlRings();
        grabSecondWobble();
        shootAdditionalRings ();
        dropSecondWobble();

//        traj6 = drive.trajectoryBuilder(new Pose2d(), false)
//                    .lineToLinearHeading(new Pose2d(-52, 11, Math.toRadians(-180)))
////                .splineToSplineHeading(new Pose2d(-52, 11, Math.toRadians(180)), Math.toRadians(0))
//                .build();
//        drive.followTrajectory(traj6);

//        Trajectory traj6 = drive.trajectoryBuilder(new Pose2d(), false)
//                .lineToLinearHeading(new Pose2d(-59, 11, -Math.toRadians(-175)))
//                .build();
//        drive.followTrajectory(traj6);

//        startShooterThread(2000, 1000);
////        startShooterThread(2000, 0.95);
//        telemetry.addData(String.format("  Shooter velocity: "), robot.shooter.getVelocity());
//        telemetry.update();
//        sleep(5000);
//        stopShooterThread();


//        traj1 = drive.trajectoryBuilder(new Pose2d(), true)
//                .lineToLinearHeading(new Pose2d(-61, 0, Math.toRadians(-2)))
//                .build();
//        drive.followTrajectory(traj1);
//        startShooterThread(2000, 1600);
//
//        robot.collection.setPower(0.5);
//        sleep(200);
//        robot.collection.setPower(0);
//        sleep(200);
//        drive.turn(Math.toRadians(-5));
//        robot.collection.setPower(0.5);
//        sleep(500);
//        robot.collection.setPower(0);
//        sleep(200);
//        drive.turn(Math.toRadians(-2));
//        robot.collection.setPower(0.5);
//        sleep(1000);
//        robot.collection.setPower(0);
//        sleep(200);
//        stopShooterThread();

//        traj1 = drive.trajectoryBuilder(new Pose2d(), true)
//                .lineToLinearHeading(new Pose2d(-61, 0, Math.toRadians(-2)))
//                .build();
//        Trajectory traj1_2 = drive.trajectoryBuilder(traj1.end(), true)
//                .lineToLinearHeading(new Pose2d(-61, 7, Math.toRadians(-2)))
//                .build();
//        Trajectory traj1_3 = drive.trajectoryBuilder(traj1_2.end(), true)
//                .lineToLinearHeading(new Pose2d(-61, 14, Math.toRadians(-2)))
//                .build();
//        drive.followTrajectory(traj1);
//        startShooterThread(2000, 1600);
//
//        robot.collection.setPower(0.5);
//        sleep(200);
//        robot.collection.setPower(0);
//        sleep(200);
//
//        drive.followTrajectory(traj1_2);
//        robot.collection.setPower(0.5);
//        sleep(500);
//        robot.collection.setPower(0);
//        sleep(200);
//
//        drive.followTrajectory(traj1_3);
//        robot.collection.setPower(0.5);
//        sleep(1000);
//        robot.collection.setPower(0);
//        sleep(200);
//        stopShooterThread();

    }

}

