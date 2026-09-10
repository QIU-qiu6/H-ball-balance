# 整理验证记录

日期：2026-09-10。

- STM32 的 CONTROL、HARDWARE、PROTOCOL、SYSTEM、USER 源文件逐文件 SHA-256 对比与原工程一致。
- 摄像头 main.py、app.yaml、app.png、mud 和 cvimodel 与原工程逐文件一致，串口协议未改变。
- Keil 工程只调整编译和列表输出目录到 `build/`。库源码随仓库打包，保持相对引用路径可解析。
- 已对整理后的 Keil 工程执行全量重编译：0 errors，0 warnings。
- 编译报告：Code=17910，RO-data=1698，RW-data=120，ZI-data=4976 字节。
- 对应用源码、配置进行了常见密钥、密码和本机用户路径模式检查，未发现匹配；这不是完整安全审计。
- 本次未做上板烧录或运动测试，也未重新运行摄像头推理。

模型 `model_297541.cvimodel` SHA-256：

```text
5B548DF5AC715C1602DBB44B8D030864CBFBBDBF909649C6F0BA0363B5E18DB4
```
