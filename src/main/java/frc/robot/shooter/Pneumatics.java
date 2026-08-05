package frc.robot.shooter;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;

public class Pneumatics {
    // Shooter（射擊機構）氣動電源控制電磁閥
    private final Solenoid m_shooterPowerSolenoid;

    public Pneumatics() {
        // 初始化 REV Pneumatic Hub 上的通道 (假設使用通道 2)
        m_shooterPowerSolenoid = new Solenoid(PneumaticsModuleType.REVPH, 2);
    }

    /**
     * 控制 Shooter 的電源開關。
     * 
     * @param enabled true 為通電/開啟電源，false 為斷開/關閉電源
     */
    public void setShooterPower(boolean enabled) {
        // 透過 PH 埠號切換輸出狀態以驅動電源
        m_shooterPowerSolenoid.set(enabled);
    }

    /**
     * 取得目前 Shooter 電源通電狀態
     */
    public boolean isShooterPowered() {
        return m_shooterPowerSolenoid.get();
    }
}
