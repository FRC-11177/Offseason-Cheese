package frc.robot.indexer;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;

public class Pneumatics {
    // Indexer（進彈/載彈機構）氣動電源控制電磁閥
    private final Solenoid m_indexerPowerSolenoid;

    public Pneumatics() {
        // 初始化 REV Pneumatic Hub 上的通道 (假設使用通道 1)
        m_indexerPowerSolenoid = new Solenoid(PneumaticsModuleType.REVPH, 1);
    }

    /**
     * 控制 Indexer 的電源開關。
     * 
     * @param enabled true 為通電/開啟電源，false 為斷開/關閉電源
     */
    public void setIndexerPower(boolean enabled) {
        // 透過 PH 埠號切換輸出狀態以驅動電源
        m_indexerPowerSolenoid.set(enabled);
    }

    /**
     * 取得目前 Indexer 電源通電狀態
     */
    public boolean isIndexerPowered() {
        return m_indexerPowerSolenoid.get();
    }
}
