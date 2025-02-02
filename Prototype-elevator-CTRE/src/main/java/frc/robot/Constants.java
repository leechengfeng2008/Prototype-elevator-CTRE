package frc.robot;

public final class Constants {
        //CAN ID
        public static final int elevMotor_RID = 0;
        public static final int elevMotor_LID = 1;

        //PID Value
        public static final double elevator_KP = 1;
        public static final double elevator_KI = 0;
        public static final double elevator_KD = 0.25;
        public static final double elevator_KS = 0.22972;
        public static final double elevator_KV = 8.567;
        public static final double elevator_KA = 0.3064;
        public static final double elevator_KG = 0.099541;

        public static final double elevatorCurrentLimit = 35;
        public static final double upVoltageCompensation = 12;
        public static final float elevator_Max_Length = 139f/9;
        public static final float elevator_Min_Length = 1f/9;

        public static final double elevator_Motor_Factor = 0.111;
        public static final double pulleyCircumference_m = 124.47*0.001;
        public static final double m_sensorToMechanismRatio = 9.0;
    
    
}
