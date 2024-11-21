package org.firstinspires.ftc.teamcode.TriSwerve;

import com.amarcolini.joos.command.Component;
import com.amarcolini.joos.command.Robot;
import com.amarcolini.joos.dashboard.JoosConfig;
import com.amarcolini.joos.hardware.IMUAngleSensor;
import com.amarcolini.joos.hardware.drive.DriveComponent;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

import java.util.List;

//TODO: change all IDs to what they are in your robot configuration
@JoosConfig
public class TriRobot extends Robot {
    public final IMUAngleSensor headingSensor = new IMUAngleSensor(
            hMap, "imu",
            RevHubOrientationOnRobot.LogoFacingDirection.UP, // TODO: Set these to what they are on your robot
            RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
    );

    //TODO: Uncomment if using 3-wheel odometry
//    public static double lateralDistance = 1.0;
//    public static double forwardOffset = 1.0;

    //TODO: Uncomment if using 2-wheel odometry
//    public static double parallelOffset = 1.0;
//    public static double perpendicularOffset = 1.0;

    //TODO: Change to your specific drivetrain
    public final TriSwerveDrive drive = new TriSwerveDrive(hMap);

    @Override
    public void init() {
        register(drive);

        /**
         * The `SwerveConstraints` variable in SampleSwerveDrive is primarily used in `SwerveLocalizer`
         * By using 3 dead-wheel odometry, we can (hopefully) localize the robot without worrying about the constraints
         */
        //TODO: Uncomment if using 3-wheel odometry
//        drive.setLocalizer(new Standard3WheelLocalizer(
//                new Motor.Encoder(hMap, "left_encoder"), //Change ids to match robot configuration
//                new Motor.Encoder(hMap, "right_encoder"), //Use .reversed() as needed
//                new Motor.Encoder(hMap, "perpendicular_encoder"),
//                lateralDistance,
//                forwardOffset
//        ));

        //TODO: Uncomment if using 2-wheel odometry
//        drive.setLocalizer(new Standard2WheelLocalizer(
//                new Motor.Encoder(hMap, "parallel_encoder"), //Change ids to match robot configuration
//                parallelOffset,
//                new Motor.Encoder(hMap, "perpendicular_encoder"), //Use .reversed() as needed
//                perpendicularOffset,
//                headingSensor
//        ));

        //Enables bulk reads
        List<LynxModule> modules = hMap.getAll(LynxModule.class);
        for (LynxModule module : modules) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
        register(Component.of(() -> {
            for (LynxModule module : modules) {
                module.clearBulkCache();
            }
        }));
    }
}