package org.firstinspires.ftc.teamcode.TriSwerve;

import com.amarcolini.joos.control.DCMotorFeedforward;
import com.amarcolini.joos.control.PIDCoefficients;
import com.amarcolini.joos.dashboard.Immutable;
import com.amarcolini.joos.dashboard.JoosConfig;
import com.amarcolini.joos.drive.AbstractSwerveDrive;
import com.amarcolini.joos.drive.SwerveModule;
import com.amarcolini.joos.followers.HolonomicPIDVAFollower;
import com.amarcolini.joos.followers.TrajectoryFollower;
import com.amarcolini.joos.geometry.Angle;
import com.amarcolini.joos.geometry.Pose2d;
import com.amarcolini.joos.geometry.Vector2d;
import com.amarcolini.joos.hardware.MotorGroup;
import com.amarcolini.joos.hardware.drive.DriveTrajectoryFollower;
import com.amarcolini.joos.hardware.drive.FollowTrajectoryCommand;
import com.amarcolini.joos.trajectory.Trajectory;
import com.amarcolini.joos.trajectory.constraints.SwerveConstraints;
import com.amarcolini.joos.trajectory.constraints.TrajectoryConstraints;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.SampleSwerveModule;
import org.firstinspires.ftc.teamcode.tuning.util.DashboardTrajectoryCommand;
import org.jetbrains.annotations.NotNull;

import java.util.Arrays;
import java.util.List;
import java.util.stream.Collectors;

@JoosConfig
public class TriSwerveDrive extends AbstractSwerveDrive implements DriveTrajectoryFollower {
    //TODO Update swerve trackwidth/wheelbase to match your drive configuration.
    private final List<Vector2d> modulePositions;
    private final List<SampleSwerveModule> modules;
    private final MotorGroup motorGroup;

    @NotNull
    @Override
    public MotorGroup getMotors() {
        return motorGroup;
    }

    // Leaving trackWidth and wheelBase at 18.0
    @Immutable
    public static SwerveConstraints constraints = new SwerveConstraints(
            40.0,
            18.0,
            18.0,
            40.0,
            40.0,
            Angle.deg(180.0),
            Angle.deg(180.0)
    );

    //TODO Tune swerve module offsets.
    public static Angle frontOffset = Angle.deg(0);
    public static Angle backLeftOffset = Angle.deg(0);
    public static Angle backRightOffset = Angle.deg(0);

    public static final PIDCoefficients axialCoeffs = new PIDCoefficients();
    public static final PIDCoefficients lateralCoeffs = new PIDCoefficients();
    public static final PIDCoefficients headingCoeffs = new PIDCoefficients();
    private final HolonomicPIDVAFollower trajectoryFollower = new HolonomicPIDVAFollower(
            axialCoeffs, lateralCoeffs, headingCoeffs,
            new Pose2d(0.5, 0.5, Angle.deg(5)), 0.5
    );
    public static final DCMotorFeedforward feedforward = new DCMotorFeedforward();
    public static double distancePerTick = 1.0;

    public TriSwerveDrive(HardwareMap hMap) {
        this(
                Arrays.asList(
                        new SampleSwerveModule(
                                hMap,
                                "front_motor",
                                "front_servo",
                                "front_angle", frontOffset
                        ),
                        new SampleSwerveModule(
                                hMap,
                                "back_left_motor",
                                "back_left_servo",
                                "back_left_angle", backLeftOffset
                        ),
                        new SampleSwerveModule(
                                hMap,
                                "back_right_motor",
                                "back_right_servo",
                                "back_right_angle", backRightOffset
                        )
                ),
                Arrays.asList(
                        new Vector2d(7.487, 0.0),
                        new Vector2d(-4.316, 6.118),
                        new Vector2d(-4.597, -5.910)
                )
        );
        //TODO Reverse motors/servos/angle sensors like this:
//        modules.get(0).motor.setReversed(true);
//        modules.get(1).servo.setReversed(true);
//        modules.get(2).moduleOrientationSensor.setReversed(true);
    }

    public TriSwerveDrive(
            @NotNull List<SampleSwerveModule> modules,
            @NotNull List<Vector2d> modulePositions
    ) {
        // super(front, backLeft, backRight, trackWidth, wheelBase);
        super(modules, modulePositions);
        this.modules = modules;
        this.modulePositions = modulePositions;
        this.motorGroup = new MotorGroup(
                modules.stream().map((i) -> i.motor).collect(Collectors.toList())
        );
        getMotors().setDistancePerTick(distancePerTick);
        getMotors().setFeedforward(feedforward);
    }

    @Override
    public void update() {
        for (SwerveModule module : modules) {
            module.update();
        }
    }

    @NotNull
    @Override
    public TrajectoryConstraints getConstraints() {
        return constraints;
    }

    @NotNull
    @Override
    public TrajectoryFollower getTrajectoryFollower() {
        return trajectoryFollower;
    }

    @NotNull
    @Override
    public FollowTrajectoryCommand followTrajectory(@NotNull Trajectory trajectory) {
        return new DashboardTrajectoryCommand(trajectory, trajectoryFollower, this);
    }
}