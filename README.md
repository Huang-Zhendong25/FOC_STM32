# FOC_STM32

基于 STM32 HAL 库的 PMSM 磁场定向控制（FOC）学习工程。项目从零搭建，按阶段推进：先做电压开环，再逐步实现电流采样、编码器角度、电流闭环和速度闭环。

## 硬件与软件环境

- MCU：STM32F407VET6
- 开发环境：STM32CubeMX + Keil MDK-ARM V5
- 电机驱动板：INMOP FOC 开发板
- 电机：带编码器的 PMSM（极对数 10，编码器 1024 PPR）
- 调试工具：ST-Link / 串口 / VOFA+

## 仓库结构

```text
foc_stm32/
├── Core/          CubeMX 生成的外设初始化和主程序
├── Drivers/       STM32F4 HAL 库和 CMSIS
├── App/           应用层（按键、VOFA+ 串口输出）
├── MotorFoc/      FOC 算法与控制（SVPWM、Clarke、开环控制）
├── Sdrive/        底层驱动（ADC 电流采样）
├── picture/       调试波形图片
├── MDK-ARM/       Keil 工程文件
├── STM32_FOC.ioc  CubeMX 工程配置
└── README.md
```

## 当前阶段总览

本 README 按阶段累积更新，不会用新阶段覆盖前面阶段的说明。

### 阶段 0：HAL 工程建立

- 使用 STM32CubeMX 生成 STM32F407VET6 基础工程。
- 配置时钟为 168MHz。
- 配置 LED、串口、ADC、TIM1、TIM3、TIM4 等外设。
- 通过点亮 LED 验证工程编译和烧录无误。

### 阶段 1：SVPWM 电压开环控制

当前已实现阶段 1，目标是验证 TIM1 三相 PWM 和 SVPWM 能正确驱动电机。

#### 实现了什么

- TIM1 输出三相互补 PWM，中心对齐模式。
- 实现七段式 SVPWM，将 αβ 轴电压矢量转换为三相比较值 CCR。
- 支持两种开环测试模式：
  - 锁轴测试：固定电压矢量，使转子吸附在某一电角度。
  - 开环旋转测试：电压矢量按固定电频率旋转，带动电机低速旋转。
- 加入按键控制：
  - KEY1：使能电机。
  - KEY2：失能电机。
  - KEY3：锁轴模式下切换 α 轴和 β 轴锁定方向。
- 启动 PWM 前先将三路 CCR 置于 50% 中点，确保从零电压矢量安全启动。

#### 实现原理

阶段 1 的信号链为：

```text
开环电角度 θ
→ Vα = U_amp · cosθ
→ Vβ = U_amp · sinθ
→ SVPWM_Update(Vα, Vβ)
→ CCR1/CCR2/CCR3
→ TIM1 三相互补 PWM
→ 三相桥 → 电机
```

其中：

- αβ 是定子静止坐标系，α 轴通常与 A 相绕组轴线重合，β 轴超前 α 轴 90°。
- `SVPWM_Update()` 先根据 αβ 电压计算三个中间量 `u1/u2/u3`，它们本质上是三路线电压除以 √3。
- 根据 `u1/u2/u3` 的正负判断电压矢量位于 6 个扇区中的哪一个。
- 确定扇区后，计算三相调制波 `Ta/Tb/Tc`。
- 以 `Vbase = Vdc/√3` 为基准电压，将电压量纲的 `Ta/Tb/Tc` 转换为标幺调制比 `m`。
- 最后通过 `CCR = m × TIM1_PERIOD_HALF + TIM1_PERIOD_HALF` 映射到定时器比较值。
- `CCR = 4200` 对应 50% 占空比，即零电压矢量。

#### 主要参数

| 参数 | 值 | 说明 |
|---|---|---|
| TIM1 周期 | 8400 | 中心对齐计数周期 |
| TIM1 中点 | 4200 | 50% 占空比，零矢量 |
| 死区 | 100 | TIM1 计数单位，约 595ns |
| 母线基准电压 | 12V | 标称值 |
| SVPWM 基准电压 | Vdc/√3 | 约 6.928V |
| 开环电压幅值 | 1.0V | 可在宏中调整 |
| 开环电频率 | 1Hz | 可在宏中调整 |

#### 代码分布

- `Core/Src/main.c`：只负责外设初始化和按键到控制模式的调度。
- `MotorFoc/foc.c/.h`：SVPWM 计算、TIM1 PWM 使能/关闭、开环锁轴与旋转控制。
- `App/key_app.c/.h`：KEY1/2/3 的初始化、消抖扫描和事件获取。

#### 如何测试阶段 1

锁轴测试：

1. 保持 `MotorFoc/foc.h` 中的 `FOC_STAGE1_LOCK_TEST` 为 1。
2. 编译烧录。
3. 按 KEY1，电机应被吸住并保持。
4. 按 KEY3，观察转子在 α 轴和 β 轴锁定位置之间切换。
5. 按 KEY2 停止。

开环旋转测试：

1. 将 `MotorFoc/foc.h` 中的 `FOC_STAGE1_LOCK_TEST` 改为 0。
2. 重新编译烧录。
3. 按 KEY1，电机开始低速旋转。
4. 按 KEY2 停止。

测试时请使用限流电源，并确认电机无负载或负载安全。

### 阶段 2：ADC 相电流采样与 Clarke 变换

阶段 2 在阶段 1 的基础上加入电流采样链，目标是得到真实三相电流和 αβ 轴电流。

#### 实现了什么

- ADC1 规则组采样 CH14、CH10、CH4，用于温度、母线电压和 VR。
- ADC1 注入组采样 CH15、CH9，分别对应 U 相和 W 相电流。
- 使用 TIM1_CH4 在 PWM 中心触发注入组采样，降低开关噪声。
- 电流零点在 PWM 输出零矢量期间标定，再按传感器增益换算为安培。
- 根据 `Iu + Iv + Iw = 0` 重建 V 相电流。
- 实现等幅值 Clarke 变换，得到 `Ialpha/Ibeta`。
- 通过 VOFA+ 非阻塞串口输出，观察三相电流和 αβ 电流波形。

#### 代码分布

- `Sdrive/current_sense.c/.h`：规则组 DMA、注入组启动、零点标定、电流滤波和换算。
- `MotorFoc/clarke.c/.h`：Clarke 变换。
- `App/vofa_app.c/.h`：VOFA+ JustFloat 非阻塞串口发送。
- `Core/Src/main.c`：初始化顺序和主循环调度。

#### PWM 中心同步 ADC 采样

当前使用 STM32F407 的 ADC 注入组实现 PWM 中心同步采样：

- TIM1_CH4 配置为 PWM Generation No Output，`Pulse = 4199`。
- ADC1 注入组触发源选择 `TIM1_CC4`，上升沿触发。
- U/W 相电流由注入组采样，Vbus 等慢速信号留在规则组连续采样。
- 上电后先以零电压矢量启动 PWM，完成电流零点标定，再关闭 PWM 等待 KEY1。

#### VOFA+ 串口输出

- 波特率：115200
- 协议：JustFloat
- 通道顺序：Iu、Iv、Iw、Ialpha、Ibeta、Vbus
- 每 5ms 尝试发送一帧，使用中断发送，不阻塞控制循环

#### 波形对比

普通 ADC 连续采样与 PWM 中心同步采样的对比如下。

三相电流波形：

![三相电流普通采样](picture/current_sense_abc.png)

![三相电流 PWM 中心同步采样](picture/current_sense_abc_pwmcenter.png)

αβ 轴电流波形：

![αβ电流普通采样](picture/current_sense_alpha-beta.png)

![αβ电流 PWM 中心同步采样](picture/current_sense_alpha-beta-pwmcenter.png)

可以看到，PWM 中心同步采样明显减少了开关瞬间的毛刺噪声。

#### 如何测试阶段 2

1. 保持阶段 1 的锁轴或开环旋转测试配置。
2. 编译烧录后，在调试器中观察 `g_current`：

```text
Iu
Iv
Iw
Ialpha
Ibeta
Vbus
```

3. 使用 VOFA+ 查看三相电流和 αβ 电流波形。
4. PWM 未使能时，三相电流应接近 0。
5. 按 KEY1 进入锁轴或旋转状态后，三相电流应为正弦波，αβ 轨迹应为圆形。

### 阶段 3：编码器角度读取与电角度计算

阶段 3 使用 TIM3 编码器接口读取转子角度，为后续 Park 变换提供电角度。

#### 实现了什么

- TIM3 Encoder Mode 四倍频解码。
- 计算每转 4096 计数的机械角度。
- 根据极对数 10 计算电角度。
- 处理计数器 0/4095 回绕。
- 使用 M 法估算机械转速。
- 上电时通过 α 轴电压矢量锁轴，完成编码器零点与电角度零点对齐。
- 通过 VOFA+ 输出机械角度、电角度和速度波形。

#### 编码器参数

- 编码器 PPR：1024
- 每转计数 CPR：4096
- 电机极对数：10
- 机械一圈对应电角度 10 个 2π

#### 角度波形

![编码器角度波形](picture/encoder.png)

#### 如何测试阶段 3

1. 编译烧录，等待上电自动对齐。
2. 手动转电机一圈：
   - `mech_angle_rad` 从 0 增大到 2π 一次。
   - `elec_angle_rad` 变化 10 次。
3. 在 VOFA+ 中观察角度和速度。
4. 若角度方向与预期相反，只改角度符号或交换 A/B 接线，不要同时改两处。

### 阶段 4：有感电流闭环 FOC

阶段 4 在电流采样和编码器角度基础上闭合电流环，实现对 Id/Iq 的直接控制。

#### 实现了什么

- 新增通用 PI 控制器，带积分限幅和输出限幅。
- 新增 Park 变换与逆 Park 变换。
- 新增电流闭环调度，运行在 ADC 注入转换完成中断中。
- Id 环目标固定为 0，Iq 环目标可调。
- 使用例程电流 PI 参数作为初始值：Kp = 0.3，Ki = 1000。
- KEY1 使能电流环，KEY2 失能电流环。
- KEY3 单击增加 Iq_ref，双击减小 Iq_ref。
- VOFA+ 可查看 Id、Iq、Vd、Vq、Iq_ref 等闭环变量。

#### 电流闭环链路

```text
ADC 注入组读取 Iu/Iw
→ 电流换算与滤波
→ Clarke 得到 Ialpha/Ibeta
→ Park 使用编码器电角度得到 Id/Iq
→ Id PI：Id_ref = 0
→ Iq PI：Iq_ref = 目标转矩电流
→ 逆 Park 得到 Valpha/Vbeta
→ SVPWM 更新 TIM1 CCR
```

#### 代码分布

- `MotorFoc/pi.c/.h`：通用 PI 控制器。
- `MotorFoc/park.c/.h`：Park 与逆 Park 变换。
- `MotorFoc/foc_current.c/.h`：电流闭环调度和 ADC 注入完成回调。
- `Sdrive/current_sense.c/.h`：注入组电流读取和换算。
- `Sdrive/encoder.c/.h`：电角度读取。
- `App/key_app.c/.h`：单击/双击按键。
- `App/vofa_app.c/.h`：闭环变量串口输出。

#### 当前 Iq 参数

```text
Iq_ref 默认值：0.2A
Iq 单击步进：0.05A
Iq 范围：±2.0A
```

#### VOFA+ 通道

当前每帧发送 16 个 float：

```text
Iu
Iv
Iw
Ialpha
Ibeta
Vbus
mech_angle_rad
elec_angle_rad
speed_rpm
Id
Iq
Vd
Vq
Iq_ref
speed_ref_rpm
speed_fb_rpm
```

#### 如何测试阶段 4

1. 堵住电机轴，避免空载持续加速。
2. 按 KEY1 启动电流环。
3. 按 KEY3 单击增加 Iq_ref，双击减小 Iq_ref。
4. 观察 VOFA+：

```text
Id 应接近 0
Iq 应接近 Iq_ref
```

5. 如果 Iq 方向反了，修改 `Sdrive/current_sense.c` 中 U/W 电流符号，两相同时修改。

### 阶段 5：速度闭环控制

阶段 5 在电流环外层加入速度 PI 环，使实际转速跟随目标转速。

#### 实现了什么

- 新增速度环模块 `MotorFoc/foc_speed.c/.h`。
- 速度环在 TIM1 更新中断中执行，先于电流环。
- 使用 M 法从编码器计数计算实际机械转速。
- 速度反馈经过一阶低通滤波。
- 目标速度带 500 rpm/s 的斜坡，避免启动冲击。
- 速度 PI 输出 Iq_ref，交给电流环。
- KEY1 使能速度环，KEY2 失能速度环。
- KEY3 单击增加目标速度，双击减小目标速度。

#### 速度环参数

```text
Kp = 0.0005
Ki = 0.01
Ts = 0.0001s
输出限幅 = ±2A
默认目标速度 = 300 rpm
速度步进 = 100 rpm
最大速度 = 3000 rpm
```

#### 速度环链路

```text
TIM1 更新中断
→ FOC_Speed_Run
    → 读取 TIM3->CNT
    → M 法计算转速
    → 低通滤波
    → 目标速度斜坡
    → 速度 PI
    → 输出 Iq_ref
→ FOC_Current_Run
    → 电流环执行
```

#### 速度波形

从 0 rpm 增加到 1000 rpm 的速度曲线：

![速度闭环 0 到 1000](picture/speed_pi.png)

将曲线放大到速度 800 rpm 附近，查看实际转速与目标转速的差别：

![速度闭环 800 附近放大](picture/speed_pi1.png)

#### 如何测试阶段 5

1. 先确认电流环稳定。
2. 编译烧录。
3. 按 KEY1 启动速度环。
4. 按 KEY3 单击增加目标速度，双击减小目标速度。
5. 在 VOFA+ 中观察：

```text
speed_ref_rpm
speed_fb_rpm
Iq
```

6. 正常现象：

```text
实际速度能跟踪目标速度；
启动电流平稳；
稳态误差接近 0。
```

### 阶段 6：位置闭环控制

阶段 6 在速度环外层加入位置环，使电机轴到达并保持在目标位置。

#### 实现了什么

- 新增位置环模块 `MotorFoc/foc_position.c/.h`。
- 位置环使用编码器计数作为位置反馈。
- 处理编码器 0/4095 回绕，选择最短路径运动。
- 位置环只用 P 控制，输出速度给定并限幅。
- 位置环在 1kHz 主循环中执行。
- KEY1 使能位置环，KEY2 失能位置环。
- KEY3 单击增加目标位置，双击减小目标位置。
- VOFA+ 增加 position_target、position_fbk、position_error。

#### 位置环参数

```text
Kp = 0.3
输出限幅 = ±3000 rpm
位置步进 = 100 counts
最大目标位置 = 3999
```

#### 位置环链路

```text
目标位置
→ 位置误差（处理编码器回绕）
→ 0.3 × 位置误差
→ 速度给定（限幅 ±3000 rpm）
→ 速度环
→ 电流环
```

#### 位置波形

连续几次调节位置时，目标位置与实际位置曲线：

![位置闭环曲线](picture/position.png)

#### 如何测试阶段 6

1. 确认速度环运行正常。
2. 编译烧录。
3. 按 KEY1 使能位置环。
4. 按 KEY3 单击/双击调整目标位置。
5. 观察电机是否平滑运动到目标位置并停止。
6. 在 VOFA+ 中观察：

```text
position_target
position_fbk
position_error
speed_ref_rpm
speed_fb_rpm
```

正常现象：

- 位置误差大时速度较高，接近目标时速度降低。
- 稳态位置误差接近 0。
- 无大幅超调。
- 正反方向都能正确运动。

## 后续阶段计划

- 后续优化：位置环积分、速度前馈、弱磁、无感观测器等。

每个阶段提交时，都会在本 README 中追加对应阶段的实现说明，并保留前面阶段的说明。
