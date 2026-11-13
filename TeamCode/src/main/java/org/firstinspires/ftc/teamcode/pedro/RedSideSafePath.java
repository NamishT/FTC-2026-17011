package org.firstinspires.ftc.teamcode.pedro;

import static com.pedropathing.api.Paths.curve;
import static com.pedropathing.api.Paths.line;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;


public class RedSideSafePath {
    private Follower follower;
    private final PoseFactory poseFactory = PoseFactory.degrees();
    private final Pose start = poseFactory.of(4.9368, 23.4104, 0);
    private final Pose point1 = poseFactory.of(57.8264, 23.434, 90);
    private final Pose point1Control1 = poseFactory.of(34.9328, 32.771, 0);
    private final Pose point2Start = poseFactory.of(57.8264, 23.434, 90);
    private final Pose point2 = poseFactory.of(28.6549, 3.7948, 0);
    private final Pose point3 = poseFactory.of(5.4191, 4.0229, -0.5623);
    private final Pose point4Start = poseFactory.of(5.4191, 4.0229, 0);
    private final Pose point4 = poseFactory.of(9.6678, 98.7754, 90);
    private final Pose point4Control1 = poseFactory.of(41.0252, 58.7618, 0);

    public Path path1() {
        return Paths.curve(start, point1Control1, point1).linear(start, point1);
    }

    public Path path2() {
        return Paths.line(point2Start, point2).linear(point2Start, point2);
    }

    public Path path3() {
        return Paths.line(point2, point3).reverseTangent();
    }

    public Path path4() {
        return Paths.curve(point4Start, point4Control1, point4).linear(point4Start, point4);
    }


    private Command autoRoutine(){
        return sequential(
                follow(follower, path1()),

                follow(follower, path2()),

                follow(follower, path3()),

                follow(follower, path4())
        );
    }

    @Autonomous
    public class First extends OpMode {
        private Follower follower;
        @Override
        public void init() {
            Scheduler.reset();
            follower = Constants.create(hardwareMap);
            follower.setPose(start);
            follower.update();
        }
        @Override
        public void start() {
            schedule(follow(follower, path1()));
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
