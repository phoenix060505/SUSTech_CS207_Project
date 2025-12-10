# ----------------------------------------------------------------------------
# 1. 时钟信号 (System Clock) - P17 (100MHz)
# ----------------------------------------------------------------------------
set_property PACKAGE_PIN P17 [get_ports clk]
set_property IOSTANDARD LVCMOS33 [get_ports clk]

# ----------------------------------------------------------------------------
# 2. 复位按键 (Reset) - 使用 S4 (U4)
# EGO1 按键按下为高电平(1)，松开为低电平(0)
# 代码中已做取反处理，使其适配低电平复位逻辑
# ----------------------------------------------------------------------------
set_property PACKAGE_PIN U4 [get_ports rst_n_in]
set_property IOSTANDARD LVCMOS33 [get_ports rst_n_in]

# ----------------------------------------------------------------------------
# 3. 启动计算按键 (Start) - 使用 S0 (R11)
# ----------------------------------------------------------------------------
set_property PACKAGE_PIN R11 [get_ports btn_s0]
set_property IOSTANDARD LVCMOS33 [get_ports btn_s0]

# ----------------------------------------------------------------------------
# 3.1 返回主菜单按键 (Back) - 使用 S2 (R15)
# ----------------------------------------------------------------------------
set_property PACKAGE_PIN R15 [get_ports btn_s2]
set_property IOSTANDARD LVCMOS33 [get_ports btn_s2]

# ----------------------------------------------------------------------------
# 4. 操作模式选择开关
# SW0 (R1) + SW1 (N4) 组成 2-bit 操作模式选择
# 00: 加法, 01: 转置, 10: 标量乘, 11: 矩阵乘
# ----------------------------------------------------------------------------
set_property PACKAGE_PIN R1 [get_ports sw0_op]
set_property IOSTANDARD LVCMOS33 [get_ports sw0_op]

set_property PACKAGE_PIN N4 [get_ports sw1_op]
set_property IOSTANDARD LVCMOS33 [get_ports sw1_op]

# ----------------------------------------------------------------------------
# 4.1 功能选择开关 (顶层菜单)
# SW6 (P4) + SW7 (P5) 组成 2-bit 功能选择
# 00: 矩阵输入, 01: 矩阵生成, 10: 矩阵展示, 11: 矩阵运算
# ----------------------------------------------------------------------------
set_property PACKAGE_PIN P4 [get_ports sw6_func]
set_property IOSTANDARD LVCMOS33 [get_ports sw6_func]

set_property PACKAGE_PIN P5 [get_ports sw7_func]
set_property IOSTANDARD LVCMOS33 [get_ports sw7_func]

# ----------------------------------------------------------------------------
# 5. UART 串口接口
# 根据 EGO1 手册：
# "UART RX" (Schematic) -> N5 (FPGA PIN) -> 这是 FPGA 的接收端 (RX)
# "UART TX" (Schematic) -> T4 (FPGA PIN) -> 这是 FPGA 的发送端 (TX)
# ----------------------------------------------------------------------------
set_property PACKAGE_PIN N5 [get_ports uart_rx]
set_property IOSTANDARD LVCMOS33 [get_ports uart_rx]

set_property PACKAGE_PIN T4 [get_ports uart_tx]
set_property IOSTANDARD LVCMOS33 [get_ports uart_tx]

# ----------------------------------------------------------------------------
# 6. LED 状态指示灯 (用于调试)
# LED0 (K3): 系统电源/复位状态 (常亮 = 正常运行)
# LED1 (M1): 串口数据接收指示 (收到数据时翻转/闪烁)
# LED2 (L1): 状态机就绪 (Ready)
# LED7 (K1): 发送忙 (Tx Busy)
# ----------------------------------------------------------------------------
set_property PACKAGE_PIN K3 [get_ports {led[0]}]
set_property IOSTANDARD LVCMOS33 [get_ports {led[0]}]

set_property PACKAGE_PIN M1 [get_ports {led[1]}]
set_property IOSTANDARD LVCMOS33 [get_ports {led[1]}]

set_property PACKAGE_PIN L1 [get_ports {led[2]}]
set_property IOSTANDARD LVCMOS33 [get_ports {led[2]}]

set_property PACKAGE_PIN K6 [get_ports {led[3]}]
set_property IOSTANDARD LVCMOS33 [get_ports {led[3]}]

set_property PACKAGE_PIN J5 [get_ports {led[4]}]
set_property IOSTANDARD LVCMOS33 [get_ports {led[4]}]

set_property PACKAGE_PIN H5 [get_ports {led[5]}]
set_property IOSTANDARD LVCMOS33 [get_ports {led[5]}]

set_property PACKAGE_PIN H6 [get_ports {led[6]}]
set_property IOSTANDARD LVCMOS33 [get_ports {led[6]}]

set_property PACKAGE_PIN K1 [get_ports {led[7]}]
set_property IOSTANDARD LVCMOS33 [get_ports {led[7]}]

# ----------------------------------------------------------------------------
# 7. 7段数码管显示 (7-Segment Display)
# ----------------------------------------------------------------------------

# ============================================================================
# DN0 组段选 (对应 seg0) - 控制右侧数码管显示的字形
# 逻辑：高电平有效 (Active High)
# ============================================================================
set_property PACKAGE_PIN B4 [get_ports {seg0[0]}]
set_property IOSTANDARD LVCMOS33 [get_ports {seg0[0]}]

set_property PACKAGE_PIN A4 [get_ports {seg0[1]}]
set_property IOSTANDARD LVCMOS33 [get_ports {seg0[1]}]

set_property PACKAGE_PIN A3 [get_ports {seg0[2]}]
set_property IOSTANDARD LVCMOS33 [get_ports {seg0[2]}]

set_property PACKAGE_PIN B1 [get_ports {seg0[3]}]
set_property IOSTANDARD LVCMOS33 [get_ports {seg0[3]}]

set_property PACKAGE_PIN A1 [get_ports {seg0[4]}]
set_property IOSTANDARD LVCMOS33 [get_ports {seg0[4]}]

set_property PACKAGE_PIN B3 [get_ports {seg0[5]}]
set_property IOSTANDARD LVCMOS33 [get_ports {seg0[5]}]

set_property PACKAGE_PIN B2 [get_ports {seg0[6]}]
set_property IOSTANDARD LVCMOS33 [get_ports {seg0[6]}]

set_property PACKAGE_PIN D5 [get_ports {seg0[7]}]
set_property IOSTANDARD LVCMOS33 [get_ports {seg0[7]}]

# ============================================================================
# DN1 组段选 (对应 seg1) - 即使不显示也需要约束，防止报错
# ============================================================================
set_property PACKAGE_PIN D4 [get_ports {seg1[0]}]
set_property IOSTANDARD LVCMOS33 [get_ports {seg1[0]}]

set_property PACKAGE_PIN E3 [get_ports {seg1[1]}]
set_property IOSTANDARD LVCMOS33 [get_ports {seg1[1]}]

set_property PACKAGE_PIN D3 [get_ports {seg1[2]}]
set_property IOSTANDARD LVCMOS33 [get_ports {seg1[2]}]

set_property PACKAGE_PIN F4 [get_ports {seg1[3]}]
set_property IOSTANDARD LVCMOS33 [get_ports {seg1[3]}]

set_property PACKAGE_PIN F3 [get_ports {seg1[4]}]
set_property IOSTANDARD LVCMOS33 [get_ports {seg1[4]}]

set_property PACKAGE_PIN E2 [get_ports {seg1[5]}]
set_property IOSTANDARD LVCMOS33 [get_ports {seg1[5]}]

set_property PACKAGE_PIN D2 [get_ports {seg1[6]}]
set_property IOSTANDARD LVCMOS33 [get_ports {seg1[6]}]

set_property PACKAGE_PIN H2 [get_ports {seg1[7]}]
set_property IOSTANDARD LVCMOS33 [get_ports {seg1[7]}]

# ============================================================================
# 片选信号 (dig_sel) - 控制哪一个数码管亮
# EGO1 为共阴极，且驱动电路高电平有效 (FPGA '1' = 数码管亮)
# DN0组: DN0_K1(G2), DN0_K2(C2), DN0_K3(C1), DN0_K4(H1)
# DN1组: DN1_K1(G1), DN1_K2(F1), DN1_K3(E1), DN1_K4(G6)
# dig_sel[7:0] = {DN1_K4, DN1_K3, DN1_K2, DN1_K1, DN0_K4, DN0_K3, DN0_K2, DN0_K1}
# ============================================================================
set_property PACKAGE_PIN G2 [get_ports {dig_sel[0]}]
set_property IOSTANDARD LVCMOS33 [get_ports {dig_sel[0]}]

set_property PACKAGE_PIN C2 [get_ports {dig_sel[1]}]
set_property IOSTANDARD LVCMOS33 [get_ports {dig_sel[1]}]

set_property PACKAGE_PIN C1 [get_ports {dig_sel[2]}]
set_property IOSTANDARD LVCMOS33 [get_ports {dig_sel[2]}]

set_property PACKAGE_PIN H1 [get_ports {dig_sel[3]}]
set_property IOSTANDARD LVCMOS33 [get_ports {dig_sel[3]}]

set_property PACKAGE_PIN G1 [get_ports {dig_sel[4]}]
set_property IOSTANDARD LVCMOS33 [get_ports {dig_sel[4]}]

set_property PACKAGE_PIN F1 [get_ports {dig_sel[5]}]
set_property IOSTANDARD LVCMOS33 [get_ports {dig_sel[5]}]

set_property PACKAGE_PIN E1 [get_ports {dig_sel[6]}]
set_property IOSTANDARD LVCMOS33 [get_ports {dig_sel[6]}]

set_property PACKAGE_PIN G6 [get_ports {dig_sel[7]}]
set_property IOSTANDARD LVCMOS33 [get_ports {dig_sel[7]}]