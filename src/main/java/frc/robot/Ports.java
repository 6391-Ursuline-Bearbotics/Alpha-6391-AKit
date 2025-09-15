package frc.robot;

import com.ctre.phoenix6.CANBus;

import frc.robot.util.drivers.CanDeviceId;

public class Ports {
    /*
     * LIST OF CHANNEL AND CAN IDS
     */

    /* SUBSYSTEM CAN DEVICE IDS */

    public static final Integer CLAW_DIGITAL = 0;
    public static final CanDeviceId CLAW_ROLLER = new CanDeviceId(61, CANBus.systemCore(0));

    public static final CanDeviceId ARM_MAIN = new CanDeviceId(60, CANBus.systemCore(0));

    public static final Integer CLIMBER_DIGITAL = 2;
    public static final CanDeviceId ELEVATOR_MAIN = new CanDeviceId(51, CANBus.systemCore(0));
    public static final CanDeviceId ELEVATOR_FOLLOWER = new CanDeviceId(50, CANBus.systemCore(0));

    public static final CanDeviceId CLIMBER = new CanDeviceId(8, CANBus.systemCore(0));

}
