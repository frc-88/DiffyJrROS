package frc.robot.diffswerve;

public class RobotMap {

    /**
     * There should be an entry here for each CAN device, preferrably grouped by device type and
     * then in numerical order. Note that for CAN, ids must be unique per device type, but not
     * across types. Thus, you cannot have two SparkMax controllers with Id 0, but you can have a
     * SparkMax with Id 0 and a TalonSRX with Id 0.
     */
    
    public static class CAN {
        public static class TALONFX {
            public static final int FL_HI_FALCON = 16;
            public static final int FL_LO_FALCON = 1;
            public static final int BL_HI_FALCON = 2;
            public static final int BL_LO_FALCON = 3;
            public static final int BR_HI_FALCON = 12;
            public static final int BR_LO_FALCON = 13;
            public static final int FR_HI_FALCON = 15;
            public static final int FR_LO_FALCON = 14;
        }

        public static final int CANIFIER = 1;
    }

    /**
     * There should be an entry here for each DIO port, preferrably in numerical order. Note that
     * for DIO only one device can connect to each port, so the numbers should be unique.
     */
    public static class DIO {
        public static final int ENCODER_FL = 0;
        public static final int ENCODER_BL = 1;
        public static final int ENCODER_BR = 2;
        public static final int ENCODER_FR = 3;
    }
}
