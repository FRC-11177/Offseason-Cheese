package frc.robot.passer;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import dev.doglog.DogLog;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.shooter.Shooter;

public class Passer implements Subsystem{
    
    public TalonFX PassLeftMotor,PassRightMotor;
    public TalonFXConfiguration PassLeftConfig,PassRightConfig;
    public MotionMagicVelocityVoltage PassPID;
    public motionmagic
    public Follower RightPID;

    public static Passer inst;

    private Passer(){
        //左
        PassLeftMotor = new TalonFX(constant.PassLeftMotor);
        PassLeftConfig = new TalonFXConfiguration();
        PassPID = new MotionMagicVelocityVoltage(0);
        RightPID = new Follower(PassLeftMotor.getDeviceID(), MotorAlignmentValue.Opposed);
    
        PassLeftConfig.MotorOutput
            .withNeutralMode(NeutralModeValue.Brake)
            .withInverted(InvertedValue.CounterClockwise_Positive);
        PassLeftConfig.Feedback
            .withSensorToMechanismRatio(constant.PassGearatio);
        PassLeftConfig.withSlot0(constant.PassPID);

        PassLeftMotor.getConfigurator().apply(PassLeftConfig);
     
        //右
        PassRightMotor = new TalonFX(constant.PassRightMotor);

        register();
    }
    


        /**
     * 取得當下的線速度
     * @return 取得速度值轉換成每秒轉＊輪周（單位：每秒公尺）
     */
    public LinearVelocity getVelocity(){
        return MetersPerSecond.of(PassLeftMotor.getVelocity().getValue().in(RotationsPerSecond)*constant.PassCirc);
            //command 只是從shooter複製 尚未檢查有沒有不適應的程式

    }
    
    /**
     * 設定目標速度
     * @param target 目標（線速度）
     * @return 執行
     * 設定控制 -> 從PID讀取速度（每秒轉之每秒公尺目標除以輪周）
     */
    public Command setState(LinearVelocity target){
        return run(
            () -> {
                PassLeftMotor.setControl(PassPID.withVelocity(RotationsPerSecond.of(target.in(MetersPerSecond)/constant.PassCirc)));
                PassRightMotor.setControl(new Follower(PassLeftMotor.getDeviceID(), MotorAlignmentValue.Opposed));        
                //寫在setstate防止狀態不刷新
                    //command 只是從shooter複製 尚未檢查有沒有不適應的程式

             }
        );
    }

    
        @Override
    public void periodic(){
        DogLog.log("Passer/CurrentVelocity", getVelocity());
        DogLog.log("Passer/PowerConsume",PassLeftMotor.getSupplyCurrent().getValue().times(PassLeftMotor.getSupplyVoltage().getValue()));
    }

        public static Passer getInstance(){
        return inst = inst == null ? new Passer() : inst ;
    }

}
/**
 * 尚未寫
 * inst
 * passer 的設定速度
 * Doglog
 * 
 */