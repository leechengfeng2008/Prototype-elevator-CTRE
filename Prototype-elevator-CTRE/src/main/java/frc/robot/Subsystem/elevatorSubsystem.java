package frc.robot.Subsystem;
import frc.robot.*;

import static edu.wpi.first.units.Units.Volts;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;


import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;

public class elevatorSubsystem extends SubsystemBase{
    private final CANBus elevCanbus = new CANBus("rio");
    private final TalonFX elevMotor_R = new TalonFX(Constants.elevMotor_RID,elevCanbus) ;
    private final TalonFX elevMotor_L = new TalonFX(Constants.elevMotor_LID,elevCanbus);
    private final DutyCycleOut elevControl = new DutyCycleOut(0);
    private final ElevatorFeedforward elevatorFeedforward = new ElevatorFeedforward(Constants.elevator_KS,Constants.elevator_KG, Constants.elevator_KV);

    private final PositionDutyCycle elevPosition = new PositionDutyCycle(0).withSlot(0);   

    private final VoltageOut elevator_voltageOut = new VoltageOut(0);
    private final MutVoltage m_voltageOut = Volts.mutable(0);
    private final MutDistance m_distance = Meters.mutable(0);
    private final MutLinearVelocity m_velocity = MetersPerSecond.mutable(0);

    private final PositionVoltage elevPositionVoltage = new PositionVoltage(0).withSlot(0);
    private final PositionTorqueCurrentFOC elevPositionTorque = new PositionTorqueCurrentFOC(0).withSlot(1);
    private final NeutralOut m_brake = new NeutralOut();

//control
    private void resetToZero(){
        elevMotor_R.setPosition(0);
        elevMotor_L.setPosition(0);
    }

    public void setElevatorTargetPosition(double position){
        //elevMotor_L.setPosition(position);
        //elevMotor_R.setPosition(position);
        elevMotor_R.setControl(elevPosition.withPosition(position).withFeedForward(elevatorFeedforward.calculate(0)));
        elevMotor_L.setControl(elevPosition.withPosition(position).withFeedForward(elevatorFeedforward.calculate(0)));
    }



//set the robot
    private void configElevMotor_R(){
        TalonFXConfiguration configElevMotor_R = new TalonFXConfiguration();
        MotorOutputConfigs motorOutputElevMotor_R = new MotorOutputConfigs();
        TalonFXConfigurator elevMotor_RConfigurator = elevMotor_R.getConfigurator();

        motorOutputElevMotor_R.Inverted = InvertedValue.Clockwise_Positive;
        motorOutputElevMotor_R.NeutralMode = NeutralModeValue.Brake;

        configElevMotor_R.CurrentLimits.StatorCurrentLimit = Constants.elevatorCurrentLimit;
        configElevMotor_R.CurrentLimits.StatorCurrentLimitEnable = true;
        // configElevMotor_R.Voltage.PeakForwardVoltage = Constants.upVoltageCompensation;

        //SoftLimit
        configElevMotor_R.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Constants.elevator_Max_Length;
        configElevMotor_R.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        configElevMotor_R.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Constants.elevator_Min_Length;
        configElevMotor_R.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

        configElevMotor_R.Feedback.SensorToMechanismRatio = (Constants.m_sensorToMechanismRatio);

        configElevMotor_R.Slot0.kP = Constants.elevator_KP;
        configElevMotor_R.Slot0.kI = Constants.elevator_KI;
        configElevMotor_R.Slot0.kD = Constants.elevator_KD;

        elevMotor_RConfigurator.apply(configElevMotor_R);
        elevMotor_RConfigurator.apply(motorOutputElevMotor_R);
    }
    private void configElevMotor_L(){
        
        TalonFXConfiguration configElevator_L = new TalonFXConfiguration();
        MotorOutputConfigs motorOutputElevMotor_L = new MotorOutputConfigs();
        TalonFXConfigurator elevMotor_LConfigurator = elevMotor_L.getConfigurator();

        // motorOutputElevMotor_L.Inverted = InvertedValue.CounterClockwise_Positive;(no inverted)\
        
        motorOutputElevMotor_L.NeutralMode = NeutralModeValue.Brake;
        
        configElevator_L.CurrentLimits.StatorCurrentLimit = Constants.elevatorCurrentLimit;
        configElevator_L.CurrentLimits.StatorCurrentLimitEnable = true;

        // configElevator_L.Voltage.PeakForwardVoltage = Constants.upVoltageCompensation;

        //SoftLimit
        configElevator_L.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Constants.elevator_Max_Length;
        configElevator_L.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        configElevator_L.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Constants.elevator_Min_Length;
        configElevator_L.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

        configElevator_L.Feedback.SensorToMechanismRatio = (Constants.m_sensorToMechanismRatio);

        configElevator_L.Slot0.kP = Constants.elevator_KP;
        configElevator_L.Slot0.kI = Constants.elevator_KI;
        configElevator_L.Slot0.kD = Constants.elevator_KD;

        elevMotor_LConfigurator.apply(configElevator_L);
        elevMotor_LConfigurator.apply(motorOutputElevMotor_L);
        
    }
    
//for sysid
    private final SysIdRoutine m_SysIdRoutine=
        new SysIdRoutine(
        new SysIdRoutine.Config(),
        new SysIdRoutine.Mechanism(
            volts ->{elevMotor_L.setControl(elevator_voltageOut.withOutput(volts));
                    elevMotor_R.setControl(new Follower(Constants.elevMotor_LID, true));},
            log ->{
                log.motor("wrist-corol")
                .voltage(m_voltageOut.mut_replace(
                    elevMotor_L.getMotorVoltage().getValueAsDouble(), Volts))
                .linearPosition(m_distance.mut_replace(
                    getElevatorPosition(), Meters))
                .linearVelocity(m_velocity.mut_replace(
                    getElevatorVelocity(), MetersPerSecond));
            },
            this));

         public elevatorSubsystem(){
            setName("elevator");



            TalonFXConfiguration elevMotorconfig = new TalonFXConfiguration();
            elevMotor_L.getConfigurator().apply(elevMotorconfig);
            elevMotor_R.getConfigurator().apply(elevMotorconfig);
            configElevMotor_L();
            configElevMotor_R();

            BaseStatusSignal.setUpdateFrequencyForAll(250,
             elevMotor_L.getPosition(),
             elevMotor_L.getVelocity(),
             elevMotor_L.getMotorVoltage());

             elevMotor_L.optimizeBusUtilization();

             SignalLogger.start();
         }
        

         public Command jotStickDriveCommand(DoubleSupplier output){
            return run(()->elevMotor_L.setControl(elevControl.withOutput(output.getAsDouble())));
         }
         public Command sysIdQuasistatic(SysIdRoutine.Direction direction){
            return m_SysIdRoutine.quasistatic(direction);
         }
         public Command sysIdDymamic(SysIdRoutine.Direction direction){
            return m_SysIdRoutine.dynamic(direction);
         }


         private double getElevatorPosition(){
            return elevMotor_L.getPosition().getValueAsDouble()*Constants.pulleyCircumference_m;
         }

         private double getElevatorVelocity(){
            return elevMotor_L.getVelocity().getValueAsDouble()*Constants.pulleyCircumference_m;
         }
        
}
