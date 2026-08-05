package frc.robot;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * 全域氣壓管理子系統 (單例模式)。
 * 僅負責 REV Pneumatics Hub (REVPH) 硬體模組與打氣機 (Compressor) 的自動壓力監控。
 */
public class Pneumatics extends SubsystemBase {

    private static Pneumatics inst; // 單例模式防呆實例

    // 來源：WPILib (edu.wpi.first.wpilibj.Compressor)
    // 用途：宣告並控制 REVPH 上的打氣機與壓力感測開關 (Analog / Pressure Switch)
    private final Compressor m_compressor = new Compressor(PneumaticsModuleType.REVPH);

    /**
     * 私有建構子：初始化打氣機控制。
     * 請透過 {@link #getInstance()} 取得單例對象。
     */
    private Pneumatics() {
        // 來源：WPILib Compressor.enableDigital()
        // 用途：開啟打氣機的自動壓力開關監控（當氣壓低於設定值時自動打氣，達到 120 PSI 時自動停止）
        m_compressor.enableDigital();

        // 來源：WPILib SubsystemBase.register()
        // 用途：向 CommandScheduler 註冊此子系統，確保 periodic() 能被定期執行
        register();
    }

    /**
     * 取得目前氣壓系統的壓力開關狀態。
     * 
     * @return true 代表氣壓已足夠 (開關切斷)，false 代表氣壓不足中
     */
    public boolean getPressureSwitch() {
        // 來源：WPILib Compressor.getPressureSwitchValue()
        // 用途：讀取壓力開關的數位訊號，用於檢查氣壓是否充飽
        return m_compressor.getPressureSwitchValue();
    }

    /**
     * 手動開關打氣機運作的控制指令。
     * 
     * @param enable true 為開啟自動打氣，false 為強制關閉打氣機
     * @return 執行切換打氣機狀態的 {@link Command}
     */
    public Command setCompressorCommand(boolean enable) {
        // 來源：WPILib SubsystemBase.runOnce()
        // 用途：建立一個僅執行一次的指令，用來控制打氣機啟用與否
        return runOnce(() -> {
            if (enable) {
                m_compressor.enableDigital();
            } else {
                m_compressor.disable();
            }
        });
    }

    /**
     * 取得氣壓管理子系統單例 (Singleton Instance)。
     * 
     * @return 全域唯一的 {@link Pneumatics} 實例
     */
    public static Pneumatics getInstance() {
        inst = inst == null ? new Pneumatics() : inst;
        return inst;
    }
}
