位置(Position)
速度(Velocity)

一般的PID (沒有前綴)
基礎調教 MotionMagic
(位置)進階調教 MotionMagicExpo

DutyCycle -- 趴數 <-- 會受到系統電壓的影像
Voltage -- 電壓
TorqueCurrentFOC -- 電流

DutyCycle, Voltage 底層都是設馬達的電壓
TorqueCurrentFOC 因為 τ = kT * I, kT 馬達的扭力轉換係數(Kraken可以用getMotorKT().getValueAsDobule()) kT單位是 Nm/A，NEO可以到https://www.reca.lc/motors

