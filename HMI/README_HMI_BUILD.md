# TJC8048X270_011C HMI 工程搭建指南

## 1. 创建工程

1. 打开 **USART HMI** 编辑器
2. 文件 → 新建 → 选择型号 **TJC8048X270_011C**
3. 显示方向：横屏 (800×480)
4. 编码：GBK（支持中文）
5. 波特率：**115200**

## 2. 添加字库

- 工具 → 字库生成器
- 生成一个 **32号** 字体（主要显示用）和一个 **24号** 字体（辅助信息）
- 导入到工程资源中

## 3. 页面结构

| 页面ID | 页面名 | 用途 |
|--------|--------|------|
| 0 | boot | 启动画面，1.5秒后自动跳转 main |
| 1 | main | 主控制页（核心操作） |
| 2 | motor | 电机详细控制 |
| 3 | setting | 参数设置 |

---

## 4. Page 0: boot（启动页）

### 背景
- 放一张启动 Logo 图片（智能剥虾机）
- 或者用文字控件 t0 显示 "智能剥虾机"

### 控件

| 控件类型 | ID | 属性 |
|----------|-----|------|
| Timer | tm0 | tim=1500, en=1 |

### 事件代码

**tm0 定时器事件 (Timer Event):**
```
page main
```

---

## 5. Page 1: main（主控制页）⭐核心页面

### 布局参考 (800×480)

```
┌─────────────────────────────────────────────────────────────────────┐
│ [t_title: "智能剥虾机控制系统"]                            y=0~40  │
├────────────────────────────┬────────────────────────────────────────┤
│  剥虾计数                  │  系统状态                              │
│  n_count (大字体)          │  t_state  t_time                      │
│                            │                                y=40~150│
├────────────────────────────┴────────────────────────────────────────┤
│  [bt_start 启动]  [bt_stop 停止]  [bt_reset 清零]          y=160~260│
├────────────────────────────┬────────────────────────────────────────┤
│  DC电机速度                │  水泵                                  │
│  n_dc  j_dc                │  bt_pump  t_pump                      │
│  h_dc (滑块)               │                               y=270~380│
├────────────────────────────┼────────────────────────────────────────┤
│  步进电机                  │  [bt_more 更多设置]                    │
│  n_step  bt_step_ena       │                               y=390~470│
└────────────────────────────┴────────────────────────────────────────┘
```

### 控件清单

所有需要 MCU 读写的控件，**务必将 vscope 属性设为 global（全局）**。

| 控件类型 | objname | 属性设置 | 说明 |
|----------|---------|----------|------|
| Text | t_title | txt="智能剥虾机", font=32号, pco=WHITE | 标题 |
| Number | n_count | val=0, font=32号, pco=65504(YELLOW), vscope=global | 剥虾计数 |
| Text | t_lbl_count | txt="剥虾计数", pco=WHITE | 标签 |
| Text | t_state | txt="停机", font=24号, pco=63488(RED), vscope=global | 系统状态 |
| Text | t_time | txt="00:00:00", font=24号, pco=WHITE, vscope=global | 运行时间 |
| Button | bt_start | txt="启 动", pco=WHITE, bco=2016(GREEN) | 启动按钮 |
| Button | bt_stop | txt="停 止", pco=WHITE, bco=63488(RED) | 停止按钮 |
| Button | bt_reset | txt="清 零", pco=BLACK, bco=33840(GRAY) | 清零按钮 |
| Number | n_dc | val=0, font=24号, vscope=global | DC电机转速显示 |
| Text | t_lbl_dc | txt="DC电机 RPM", pco=WHITE | DC标签 |
| Progress Bar | j_dc | val=0, bco=0(BLACK), pco=2016(GREEN), vscope=global | DC速度条 |
| Slider | h_dc | maxval=1800, minval=0, val=0, vscope=global | DC速度滑块 |
| Number | n_step | val=0, font=24号, vscope=global | 步进转速显示 |
| Text | t_lbl_step | txt="步进电机 RPM" | 步进标签 |
| Dual-state Button | bt_step_ena | val=0, txt0="步进OFF", txt1="步进ON", bco0=33840, bco1=2016 | 步进使能 |
| Dual-state Button | bt_pump | val=0, txt0="水泵OFF", txt1="水泵ON", bco0=33840, bco1=31(BLUE) | 水泵开关 |
| Text | t_pump | txt="OFF", pco=63488(RED), vscope=global | 水泵状态 |
| Button | bt_more | txt="更多设置", bco=31(BLUE), pco=WHITE | 跳转motor页 |

### ⭐ 事件代码（按控件逐一输入）

---

#### bt_start — 按下事件 (Touch Press Event)
```
prints "RUN,1",5
printh FF FF FF
```

---

#### bt_stop — 按下事件 (Touch Press Event)
```
prints "RUN,0",5
printh FF FF FF
```

---

#### bt_reset — 按下事件 (Touch Press Event)
```
prints "RSTCNT",6
printh FF FF FF
n_count.val=0
```

---

#### h_dc — 弹起事件 (Touch Release Event)
```
prints "DCSPD,",6
prints h_dc.val,0
printh FF FF FF
n_dc.val=h_dc.val
j_dc.val=h_dc.val*100/1800
```

---

#### bt_step_ena — 按下事件 (Touch Press Event)
```
if(bt_step_ena.val==1)
{
  prints "STEPENA,1",9
  printh FF FF FF
}else
{
  prints "STEPENA,0",9
  printh FF FF FF
}
```

---

#### bt_pump — 按下事件 (Touch Press Event)
```
if(bt_pump.val==1)
{
  prints "PUMP,1",6
  printh FF FF FF
  t_pump.txt="ON"
  t_pump.pco=2016
}else
{
  prints "PUMP,0",6
  printh FF FF FF
  t_pump.txt="OFF"
  t_pump.pco=63488
}
```

---

#### bt_more — 按下事件 (Touch Press Event)
```
page motor
```

---

## 6. Page 2: motor（电机详细控制页）

### 控件清单

| 控件类型 | objname | 属性 | 说明 |
|----------|---------|------|------|
| Text | t_title2 | txt="电机控制" | 页面标题 |
| Text | t_lbl_dc2 | txt="DC电机速度(RPM)" | |
| Slider | h_dc2 | maxval=1800, minval=0 | DC速度滑块 |
| Number | n_dc2 | val=0, vscope=global | DC当前值 |
| Dual-state Button | bt_dcdir | val=0, txt0="正转", txt1="反转", bco0=2016, bco1=BROWN | DC方向 |
| Text | t_lbl_step2 | txt="步进电机速度(RPM)" | |
| Slider | h_step | maxval=5000, minval=0 | 步进速度滑块 |
| Number | n_step2 | val=0, vscope=global | 步进当前值 |
| Dual-state Button | bt_stepdir | val=0, txt0="正转", txt1="反转" | 步进方向 |
| Button | bt_back | txt="返回主页" | 返回 |

### 事件代码

---

#### h_dc2 — 弹起事件 (Touch Release Event)
```
prints "DCSPD,",6
prints h_dc2.val,0
printh FF FF FF
n_dc2.val=h_dc2.val
main.n_dc.val=h_dc2.val
main.h_dc.val=h_dc2.val
main.j_dc.val=h_dc2.val*100/1800
```

---

#### bt_dcdir — 按下事件 (Touch Press Event)
```
if(bt_dcdir.val==1)
{
  prints "DCDIR,1",7
  printh FF FF FF
}else
{
  prints "DCDIR,0",7
  printh FF FF FF
}
```

---

#### h_step — 弹起事件 (Touch Release Event)
```
prints "STEPSPD,",8
prints h_step.val,0
printh FF FF FF
n_step2.val=h_step.val
main.n_step.val=h_step.val
```

---

#### bt_stepdir — 按下事件 (Touch Press Event)
```
if(bt_stepdir.val==1)
{
  prints "STEPDIR,1",9
  printh FF FF FF
}else
{
  prints "STEPDIR,0",9
  printh FF FF FF
}
```

---

#### bt_back — 按下事件 (Touch Press Event)
```
page main
```

---

## 7. Page 3: setting（设置页）

### 控件清单

| 控件类型 | objname | 说明 |
|----------|---------|------|
| Text | t_title3 | txt="参数设置" |
| Text | t_info1 | txt="串口波特率: 115200" |
| Text | t_info2 | txt="屏幕型号: TJC8048X270" |
| Text | t_info3 | txt="固件版本: v1.0" |
| Button | bt_back3 | txt="返回主页" |

### 事件代码

#### bt_back3 — 按下事件 (Touch Press Event)
```
page main
```

---

## 8. 编译与烧录

### TF 卡烧录步骤（推荐✅）

1. 在 USART HMI 编辑器中点击 **编译**
2. 点击 **输出** → 生成 `.tft` 文件
3. 准备 **MicroSD 卡**（≤32GB，FAT32格式）
4. 将 `.tft` 文件拷贝到 SD 卡**根目录**（卡里只放这一个 .tft 文件）
5. 串口屏断电
6. 插入 SD 卡
7. 给串口屏上电
8. 等待屏幕显示进度条，烧录完成后自动重启
9. 拔掉 SD 卡

### 注意事项

- SD 卡根目录下只能有 **一个** .tft 文件
- SD 卡建议 **32GB 以下**，FAT32 格式
- 烧录过程中 **不要断电**
- 如果烧录失败，格式化 SD 卡重试

---

## 9. 联调调试技巧

### 单独测试串口屏（不接 MCU）
1. 用 USB-TTL 模块连接串口屏的 TX/RX
2. 打开串口助手，设置 115200, 8N1
3. 发送测试指令（注意加 0xFF 0xFF 0xFF 结尾）:
   - 设置计数: `main.n_count.val=42` + `FF FF FF`
   - 设置状态: `main.t_state.txt="运行中"` + `FF FF FF`
   - 跳转页面: `page main` + `FF FF FF`

### 单独测试 MCU（不接串口屏）
1. 用串口助手连接 STM32 的 USART2 (PA2/PA3)
2. 发送模拟命令（HEX 模式）:
   - 启动: `52 55 4E 2C 31 FF FF FF` (即 "RUN,1" + 0xFF×3)
   - 停止: `52 55 4E 2C 30 FF FF FF`
   - DC速度: `44 43 53 50 44 2C 36 30 30 FF FF FF` (即 "DCSPD,600")
   - 水泵开: `50 55 4D 50 2C 31 FF FF FF`

---

## 10. 通信协议速查表

### MCU → 串口屏
| 功能 | 指令 |
|------|------|
| 设置计数 | `main.n_count.val=数字\xFF\xFF\xFF` |
| 设置状态文本 | `main.t_state.txt="文本"\xFF\xFF\xFF` |
| 设置状态颜色 | `main.t_state.pco=颜色值\xFF\xFF\xFF` |
| 设置DC转速 | `main.n_dc.val=数字\xFF\xFF\xFF` |
| 设置进度条 | `main.j_dc.val=0~100\xFF\xFF\xFF` |
| 设置时间 | `main.t_time.txt="HH:MM:SS"\xFF\xFF\xFF` |
| 设置步进 | `main.n_step.val=数字\xFF\xFF\xFF` |
| 设置水泵 | `main.t_pump.txt="ON/OFF"\xFF\xFF\xFF` |
| 跳转页面 | `page 页面名\xFF\xFF\xFF` |

### 串口屏 → MCU
| 功能 | 命令 |
|------|------|
| 启动 | `RUN,1` + 0xFF×3 |
| 停止 | `RUN,0` + 0xFF×3 |
| DC速度 | `DCSPD,数字` + 0xFF×3 |
| DC方向 | `DCDIR,0或1` + 0xFF×3 |
| 步进使能 | `STEPENA,0或1` + 0xFF×3 |
| 步进速度 | `STEPSPD,数字` + 0xFF×3 |
| 步进方向 | `STEPDIR,0或1` + 0xFF×3 |
| 水泵 | `PUMP,0或1` + 0xFF×3 |
| 清零计数 | `RSTCNT` + 0xFF×3 |

### 颜色代码
| 颜色 | 值 |
|------|-----|
| RED | 63488 |
| GREEN | 2016 |
| BLUE | 31 |
| YELLOW | 65504 |
| WHITE | 65535 |
| BLACK | 0 |
| GRAY | 33840 |
| BROWN(橙) | 48192 |
