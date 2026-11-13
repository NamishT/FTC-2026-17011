/*package org.firstinspires.ftc.teamcode.pedro;
import com.pedropathing.api.Paths;
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
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import org.firstinspires.ftc.teamcode.subsystems.Drivebase;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Set;

@Autonomous
public class RedSideTiltPath {
    private Follower follower;
    private Telemetry myAutoTe;
    private Intake intake;
    private Shooter shooter;
    private Drivebase drivebase;

    private Telemetry telemetryA;
    private final PoseFactory poseFactory = PoseFactory.degrees();
    private enum AUTO_STATES {
        PATH_ACTIVE,
        START,
        INACTIVE,
        SHOOT_STATE,
        // decide future paths/states
    }
    // setting three main states cycle to start with
    private AUTO_STATES currentState = AUTO_STATES.START;
    private AUTO_STATES nextState = AUTO_STATES.INACTIVE;
    private AUTO_STATES previousState = AUTO_STATES.INACTIVE;

    @Autonomous(name = "AutoPath", group = "Autonomous")
    public class AutoPath {

        private Follower follower;

        private final PoseFactory poseFactory = PoseFactory.degrees();

        private final Pose start = poseFactory.of(10.5802, 28.6718, 0);
        private final Pose path1 = poseFactory.of(59.2027, 18.5309, 90);
        private final Pose point2 = poseFactory.of(14.6424, 4.9644, -121.6518);
        private final Pose point2Control1 = poseFactory.of(26.8252, 24.7272, 0);
        private final Pose point3 = poseFactory.of(2.5683, 4.7598, -179.0293);
        private final Pose point4Start = poseFactory.of(2.5683, 4.7598, -180);
        private final Pose point4 = poseFactory.of(57.1537, 120.3914, 270);
        private final Pose point4Control1 = poseFactory.of(24.3694, 78.3654, 0);
        private final Pose point4Control2 = poseFactory.of(24.959, 110.596, 0);
        private final Pose point5Start = poseFactory.of(57.1537, 120.3914, 270);
        private final Pose point5 = poseFactory.of(8.881, 96.0901, 90);
        private final Pose point5Control1 = poseFactory.of(9.5213, 71.3476, 0);

        // Autonomous routine
        public Command autoRoutine() {
            return sequential(
                    follow(follower, path1()),
                    follow(follower, path2()),
                    follow(follower, path3()),
                    follow(follower, path4()),
                    follow(follower, path5())
            );
        }


        public void runOpMode() {
            Scheduler.reset();
            follower = Constants.create(hardwareMap);
            follower.setPose(start);
            follower.update();

            waitForStart();
            schedule(autoRoutine());

            while (opModeIsActive()) {
                follower.update();
                Scheduler.execute();

                myAutoTe.addData("x", follower.pose().x());
                myAutoTe.addData("y", follower.pose().y());
                myAutoTe.addData("heading", follower.pose().heading());

                if (follower.currentPath() != null) {
                    myAutoTe.addData("Current path distance remaining", follower.distanceToEndpoint());
                    myAutoTe.addData("Path number", follower.pathIndex());
                }

                myAutoTe.update();
            }
        }

        public Path path1() {
            return Paths.line(start, path1).linear(start, path1);
        }

        public Path path2() {
            return Paths.curve(path1, point2Control1, point2).tangent();
        }

        public Path path3() {
            return Paths.line(point2, point3).tangent();
        }

        public Path path4() {
            return Paths.curve(point4Start, point4Control1, point4Control2, point4).linear(point4Start, point4);
        }

        public Path path5() {
            return Paths.curve(point5Start, point5Control1, point5).linear(point5Start, point5);
        }
    }


}

 */


