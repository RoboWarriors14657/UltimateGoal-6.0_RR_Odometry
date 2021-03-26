package org.firstinspires.ftc.teamcode.ftc14657;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.drive.advanced.PoseStorage;

@Autonomous(name="2-Red-HighGoal", group="OpMode")
public class Auto14657_Red_HighGoal extends Auto14657_Main_UG {
    public void runAutoActions(){

        moveToLaunchLineShoot (true, false);
        moveToTargetZone(true,false);
        collectAdditionlRings (true);
        grabSecondWobble();
        shootAdditionalRings ();
        dropSecondWobble();

        // Transfer the current pose to PoseStorage so we can use it in TeleOp
        PoseStorage.currentPose = drive.getPoseEstimate();



    }

}

