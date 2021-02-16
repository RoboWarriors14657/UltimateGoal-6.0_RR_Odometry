package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(group = "drive")
//@Disabled
public class SplineTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        waitForStart();

        if (isStopRequested()) return;

//        Trajectory traj = drive.trajectoryBuilder(new Pose2d(), false)
//                .splineTo(new Vector2d(-20, 0), Math.toRadians(0))
//                .splineTo(new Vector2d(-16.5, 20), Math.toRadians(0))
//                .build();
        Trajectory traj  = drive.trajectoryBuilder(new Pose2d(), false)
//                .splineToLinearHeading(new Pose2d(38, -25, Math.toRadians(0)),  Math.toRadians(0))
                .lineToLinearHeading(new Pose2d(78, -25, Math.toRadians(0)))
                .build();

//        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(180));
////        Trajectory traj = drive.trajectoryBuilder(new Pose2d(), true)
////                .splineTo(new Vector2d(-30, -30), Math.toRadians(180))
////                .build();
//        Trajectory traj1 = drive.trajectoryBuilder(new Pose2d(), true)
//                .lineToLinearHeading(new Pose2d(-58, 0, Math.toRadians(-15)))
//                .build();
//
//        Trajectory traj2 = drive.trajectoryBuilder(traj1.end(), true)
//                .lineToLinearHeading(new Pose2d(-86, 18, Math.toRadians(0)))
//                .build();
//        Trajectory traj3 = drive.trajectoryBuilder(traj2.end(), false)
//                .lineToLinearHeading(new Pose2d(-50, 16, Math.toRadians(0)))
//                .build();
//        Trajectory traj4 = drive.trajectoryBuilder(traj3.end(), false)
//                .lineToLinearHeading(new Pose2d(-30, 30, Math.toRadians(0)))
//                .build();
//        Trajectory traj5 = drive.trajectoryBuilder(traj4.end(), false)
//                .lineToLinearHeading(new Pose2d(-83, 18, Math.toRadians(185)))
//                .build();
//
//
//        drive.followTrajectory(traj1);
//        drive.followTrajectory(traj2);
//        drive.followTrajectory(traj3);
//        drive.followTrajectory(traj4);
//        drive.followTrajectory(traj5);

        drive.followTrajectory(traj);
//        sleep(2000);
//
//        drive.followTrajectory(
//                drive.trajectoryBuilder(traj.end(), true)
//                        .splineTo(new Vector2d(0, 0), Math.toRadians(180))
//                        .build()
//        );
    }
}
