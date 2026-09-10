# 接线与通信

| 信号 | 连接 |
| --- | --- |
| MaixCAM-Pro A19 / UART1 TX | STM32 PA3 / USART2 RX |
| MaixCAM-Pro GND | STM32 GND，共地 |
| MaixCAM-Pro A18 / UART1 RX | 当前单向通信不需要连接；STM32 PA2 发送已关闭 |
| STM32 PB10 / USART3 TX | 电机 RX |
| STM32 PB11 / USART3 RX | 电机 TX |
| OLED SCL | PB6 |
| OLED SDA | PB7 |

串口为 115200、8 位数据、无校验、1 位停止位。使用 3.3 V TTL 电平；测试 USB-TTL 接收时，只需要连接 RX 和 GND。已知此前乱码与一块损坏的 USB-TTL 模块有关。

摄像头使用稳定的 5 V 供电，电机电源按驱动器规格接入。项目曾出现共用电源引起摄像头重启，换降压模块后解决。不要把电机的 12 V 接到摄像头。

## 按键定义存在历史覆盖

`USER/main.c` 前面把 KEY_Q3 / KEY_CENTER 定义为 0x01 / 0x02，随后又通过 `#undef` 改为 0x02 / 0x01。以最后定义为准：当前保存目标的 KEY_Q3 对应 PB13，KEY_CENTER 对应 PB12。PCB 上实体 K1/K2 的标签需要按实物验证，不能仅按前面的中文注释接线。

PB14、PB15、PA8 也用于按键。PA0 的急停输入是否实际焊接连通需要确认；历史装配中用户曾说明该引脚未焊接。上述差异在本次仓库整理中只记录，不修改运行逻辑。
