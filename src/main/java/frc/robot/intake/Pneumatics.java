package frc.robot.intake;
 
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;

public class Pneumatics {
    // 將變數名稱由 m_intakeSolenoid 改為更能表達用途的名稱
    private final Solenoid m_intakePowerSolenoid;

    public Pneumatics() {
        // 初始化 REV Pneumatic Hub 上的通道 (假設使用通道 0)
        m_intakePowerSolenoid = new Solenoid(PneumaticsModuleType.REVPH, 0);
    }

    /**
     * 控制 Intake 的電源開關。
     * 
     * @param enabled true 為通電/開啟電源，false 為斷開/關閉電源
     */
    public void setIntakePower(boolean enabled) {
        // 透過 PH 埠號切換輸出狀態以驅動電源
        m_intakePowerSolenoid.set(enabled);
    }

    /**
     * 取得目前 Intake 電源通電狀態
     */
    public boolean isIntakePowered() {
        return m_intakePowerSolenoid.get();
    }
}
