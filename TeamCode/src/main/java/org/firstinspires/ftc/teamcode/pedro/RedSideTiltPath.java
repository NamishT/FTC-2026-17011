package org.firstinspires.ftc.teamcode.pedro;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.pedropathing.follower.Follower;

import static com.pedropathing.api.Paths.*;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import static com.pedropathing.ivy.groups.Groups.sequential;


public class RedSideTiltPath {
    private Follower follower;
    private final PoseFactory poseFactory = PoseFactory.degrees();


    private final Pose start = poseFactory.of(10.5802, 28.6718, 0);
    private final Pose path1 = poseFactory.of(59.2027, 18.5309, 90);
    private final Pose point2 = poseFactory.of(14.6424, 4.9644, -121.6518);
    private final Pose point2Control1 = poseFactory.of(26.8252, 24.7272, 0);
    private final Pose point3 = poseFactory.of(2.5683, 4.7598, -179.0293);
    private final Pose point4Start = poseFactory.of(2.5683, 4.7598, -180);
    private final Pose point4 = poseFactory.of(57.1537, 120.3914, 270);
    private final Pose point4Control1 = poseFactory.of(0, 92.3189, 0);
    private final Pose point5Start = poseFactory.of(57.1537, 120.3914, 270);
    private final Pose point5 = poseFactory.of(8.881, 96.0901, 90);
    private final Pose point5Control1 = poseFactory.of(9.5213, 71.3476, 0);

    public Path startToShoot() {
        return line(start, path1).linear(start, path1);
    }

    public Path shootToGarden() {
        return curve(path1, point2Control1, point2).tangent();
    }

    public Path PickUpGarden() {
        return line(point2, point3).tangent();
    }

    public Path SecondShoot() {
        return line(point4Start, point4).linear(point4Start, point4);
    }

    public Path Park() {
        return curve(point4, point5Control1, point5).linear(point4, point5);
    }
    private Command autoRoutine(){
        return sequential(
                follow(follower, startToShoot()),

                follow(follower, shootToGarden()),

                follow(follower, SecondShoot()),

                follow(follower, Park())
        );
    }

    @Autonomous
    public class First extends OpMode {
        private Follower follower;
        @Override
        public void init() {
            Scheduler.reset();
            follower = Constants.create(hardwareMap);
            follower.setPose(path1);
            follower.update();
        }
        @Override
        public void start() {
            schedule(follow(follower, startToShoot()));
        }
        @Override
        public void loop() {
            follower.update();
            Scheduler.execute();

            telemetry.addData("X", follower.pose().x());
            telemetry.addData("Y", follower.pose().y());
            telemetry.addData("Heading", Math.toDegrees(follower.pose().heading()));

        }

    }

}
