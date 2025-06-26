package frc.robot;

import frc.robot.util.drivers.CanDeviceId;

public class Ports {
    /*
     * LIST OF CHANNEL AND CAN IDS
     */

    /* SUBSYSTEM CAN DEVICE IDS */

    public static final Integer CLAW_DIGITAL = 0;
    public static final CanDeviceId CLAW_ROLLER = new CanDeviceId(61, "rio");

    public static final CanDeviceId ARM_MAIN = new CanDeviceId(60, "rio");

    public static final Integer CLIMBER_DIGITAL = 2;
    public static final CanDeviceId ELEVATOR_MAIN = new CanDeviceId(51, "rio");
    public static final CanDeviceId ELEVATOR_FOLLOWER = new CanDeviceId(50, "rio");

    public static final CanDeviceId CLIMBER = new CanDeviceId(8, "rio");

}
