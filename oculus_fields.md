# Oculus Sonar Fields (liboculus)

> 说明：以下按结构体分类整理。对厂商协议未公开语义的字段，在同一表格内进行合并描述。

## OculusMessageHeader

| 字段 | 类型 | 描述 |
|---|---|---|
| oculusId | uint16_t | 固定校验值0x4f53，用于判定包是否为Oculus协议；不匹配通常意味着错包或数据错位，建议丢弃并记录。 |
| srcDeviceId | uint16_t | 发送端设备ID，用于区分多设备来源；在多声呐或多进程环境中便于路由与日志关联。 |
| dstDeviceId | uint16_t | 接收端设备ID，标识目标设备或主机；对广播包可能为特定值，用于过滤或调试。 |
| msgId | uint16_t | 消息类型枚举值，决定负载结构解析方式；错误解析会导致后续字段错读。 |
| msgVersion | uint16_t | 消息版本号，用于兼容不同固件结构差异；可据此选择解析路径或告警。 |
| payloadSize | uint32_t | 负载字节数（不含头），用于完整性校验与跳包；截断时常见不匹配。 |
| spare2 | uint16_t | 保留字段，协议未说明用途；建议写零并忽略读取，避免版本升级出现兼容问题。 |

## OculusSimpleFireMessage (v1)

| 字段 | 类型 | 描述 |
|---|---|---|
| head | OculusMessageHeader | 内嵌消息头，提供类型与长度等元信息；用于校验并定位后续 Fire 参数字段。 |
| masterMode | uint8_t | 主模式：1低频、2高频；影响频段、量程与成像特性，不支持的模式会被设备忽略或报错。 |
| pingRate | PingRateType | 最大 ping 率枚举（低/正常/高/最高/待机）；实际速率受链路与设备状态限制。 |
| networkSpeed | uint8_t | 网络速率上限（Mbps）；用于高延迟链路降速，0或0xFF通常代表自动。 |
| gammaCorrection | uint8_t | 伽马校正参数，0或255≈1.0；影响图像对比度与可视化，不改变原始采样值。 |
| flags | uint8_t | 位标志集合：范围单位、数据位深、是否发送增益、simple return、增益辅助、512 beams等。 |
| range | double | 量程需求值，单位由 flags 决定（米或百分比）；影响范围分辨率与图像覆盖范围。 |
| gainPercent | double | 期望增益百分比，控制回波强度；过高易饱和，过低会造成图像偏暗或细节丢失。 |
| speedOfSound | double | 声速 m/s，0 表示用内部估算；决定测距与范围标定精度，建议按环境设置。 |
| salinity | double | 盐度 ppt，用于内部声速估算；淡水可设0，海水环境建议设置以提高测距精度。 |

## OculusSimpleFireMessage2 (v2)

| 字段 | 类型 | 描述 |
|---|---|---|
| head | OculusMessageHeader | 内嵌消息头，提供类型与长度等元信息；用于校验并定位后续 Fire2 参数字段。 |
| masterMode | uint8_t | 主模式：1低频、2高频；决定中心频率与量程能力，需与设备型号支持匹配。 |
| pingRate | PingRateType | 最大 ping 率枚举；用于限制帧率上限，实际速率仍受网络与设备负载影响。 |
| networkSpeed | uint8_t | 网络速率上限（Mbps），0或0xFF为自动；用于降低带宽占用或跨网段传输。 |
| gammaCorrection | uint8_t | 伽马校正参数（255≈1.0）；影响回波强度映射和视觉对比度，不改变采样值。 |
| flags | uint8_t | 位标志集合：范围单位、位深、是否带增益、simple return、增益辅助、512 beams等。 |
| rangePercent | double | 量程需求百分比（v2字段），设备内部换算为米；影响 rangeResolution 与图像覆盖。 |
| gainPercent | double | 期望增益百分比；建议结合场景动态调整，避免饱和或低对比度。 |
| speedOfSound | double | 声速 m/s，0 表示内部估算；影响测距与范围标定精度，建议按环境设置。 |
| salinity | double | 盐度 ppt，参与声速估算；海水/淡水差异会显著影响声速模型。 |
| extFlags | uint32_t | 扩展标志位（如 32-bit 模式等）；具体位含义依固件而定，未知位建议置0。 |
| reserved[8] | uint32_t[8] | 保留字段数组，协议未说明语义；建议写零并忽略读取，避免未来版本兼容问题。 |

## OculusSimplePingResult (v1)

| 字段 | 类型 | 描述 |
|---|---|---|
| fireMessage | OculusSimpleFireMessage | 触发本次 ping 的 Fire 参数，用于回溯配置（频段、量程、增益等）并解释结果。 |
| pingId | uint32_t | 自增序号，用于丢包检测、排序与对齐；设备重启后可能清零需结合时间使用。 |
| status | uint32_t | 状态位集合（固件相关）；可用于诊断异常或告警，但位定义需参考设备文档。 |
| frequency | double | 实际声学频率（Hz），由设备模式与型号决定；用于后处理算法参数与物理解释。 |
| temperature | double | 外部温度（°C），可能来自探头或外壳传感器；可用于环境监测与声速推算。 |
| pressure | double | 外部压力（bar），可换算深度；需结合校准与流体密度模型以提高精度。 |
| speedOfSoundUsed | double | 本次 ping 实际使用的声速（m/s），决定测距精度与 rangeResolution 计算结果。 |
| pingStartTime | uint32_t | 上电后的时间戳（秒），用于多源对齐；粒度较粗时建议结合系统时间记录。 |
| dataSize | DataSizeType | 样本位深枚举（8/16/24/32-bit），决定图像每像素字节数与数据解析路径。 |
| rangeResolution | double | 每个 range bin 对应的物理距离（米）；与 nRanges、量程共同决定覆盖范围。 |
| nRanges | uint16_t | range 行数，决定图像纵向分辨率；与 rangeResolution 一起确定实际量程。 |
| nBeams | uint16_t | beam 数，常见256或512；决定横向分辨率与数据量、bearing 数组大小。 |
| imageOffset | uint32_t | 图像数据在包内字节偏移（从头算起），用于定位图像 payload 起点。 |
| imageSize | uint32_t | 图像数据字节长度，通常与位深×nRanges×nBeams一致（含增益时需修正）。 |
| messageSize | uint32_t | 完整消息长度（字节），用于校验读满与跳包；不匹配常见于截断或错位。 |
| bearings[] | int16_t[] | 结构体后附方位角数组（0.01°精度），用于将 beam 索引映射为实际角度。 |

## OculusSimplePingResult2 (v2)

| 字段 | 类型 | 描述 |
|---|---|---|
| fireMessage | OculusSimpleFireMessage2 | 触发本次 ping 的 Fire2 参数，用于回溯配置（频段、量程、增益等）。 |
| pingId | uint32_t | 自增序号，用于丢包检测与排序；重启后可能清零需结合时间戳使用。 |
| status | uint32_t | 状态位集合（固件相关）；用于诊断异常或告警，位语义需参考设备文档。 |
| frequency | double | 实际声学频率（Hz），由设备与模式决定；用于算法参数选择与物理解释。 |
| temperature | double | 外部温度（°C），用于环境监测；可能来自探头或外壳传感器。 |
| pressure | double | 外部压力（bar），可换算深度；需结合校准与流体密度模型。 |
| heading | double | 航向角（度），来自外部传感器或IMU；缺失时可能为0或无效值。 |
| pitch | double | 俯仰角（度），用于姿态补偿；传感器质量不佳时会影响配准效果。 |
| roll | double | 横滚角（度），用于姿态补偿；建议在日志中监测是否存在异常跳变。 |
| speedOfSoundUsed | double | 本次 ping 使用的声速（m/s），决定测距精度与 rangeResolution 结果。 |
| pingStartTime | double | 上电后时间戳（秒，微秒分辨率）；适合与多源传感器做时间对齐。 |
| dataSize | DataSizeType | 样本位深枚举，决定图像每像素字节数；影响 imageSize 校验与解析逻辑。 |
| rangeResolution | double | 每个 range bin 对应的物理距离（米），与量程需求与 nRanges 相关。 |
| nRanges | uint16_t | range 行数，决定图像纵向分辨率；与 rangeResolution 一起确定量程。 |
| nBeams | uint16_t | beam 数，决定横向分辨率与 bearing 数量；常见256或512。 |
| spare0–spare3 | uint32_t | 保留字段，协议未说明语义；建议忽略读取，避免依赖不可控的固件变化。 |
| imageOffset | uint32_t | 图像数据在包内字节偏移（从头算起），用于定位图像 payload 起点。 |
| imageSize | uint32_t | 图像数据字节长度，通常与位深×nRanges×nBeams一致（含增益时修正）。 |
| messageSize | uint32_t | 完整消息长度（字节），用于校验读满与跳包；不匹配常见于截断或错位。 |
| bearings[] | int16_t[] | 结构体后附方位角数组（0.01°精度），用于将 beam 索引映射为实际角度。 |

## OculusVersionInfo

| 字段 | 类型 | 描述 |
|---|---|---|
| firmwareVersion0 | uint32_t | ARM0 固件版本编码（高8位主/次、低16位build），用于兼容性判断与升级追踪。 |
| firmwareDate0 | uint32_t | ARM0 固件日期编码，用于追溯版本来源与发布时间；便于现场诊断与回滚。 |
| firmwareVersion1 | uint32_t | ARM1 固件版本编码，区分双核固件差异；用于诊断和对齐升级策略。 |
| firmwareDate1 | uint32_t | ARM1 固件日期编码；与版本配套用于确认实际运行固件。 |
| firmwareVersion2 | uint32_t | FPGA/bitfile 版本编码，决定硬件逻辑特性；影响协议细节与性能表现。 |
| firmwareDate2 | uint32_t | FPGA/bitfile 日期编码；用于判断硬件逻辑是否更新与兼容性风险。 |

## OculusStatusMsg

| 字段 | 类型 | 描述 |
|---|---|---|
| hdr | OculusMessageHeader | 状态广播包头部，提供类型与长度等信息；用于识别与校验该包为状态消息。 |
| deviceId | uint32_t | 设备唯一ID，用于区分网络中多个声呐；建议作为主键写入日志或数据库。 |
| deviceType | OculusDeviceType | 设备类型枚举（成像声呐）；用于上层应用识别设备类别与功能范围。 |
| partNumber | OculusPartNumberType | 型号/部件号枚举，决定频段与量程能力；可与 OculusInfo 查表关联。 |
| status | uint32_t | 设备状态位集合，语义依固件；用于判断故障、告警或工作状态变化。 |
| versionInfo | OculusVersionInfo | 固件/FPGA版本集合；用于兼容性判断、升级管理与问题追踪。 |
| ipAddr | uint32_t | 设备IP（网络字节序），用于建立控制与数据链路；需转换为点分十进制。 |
| ipMask | uint32_t | 子网掩码（网络字节序），用于网络配置校验与联通性判断。 |
| connectedIpAddr | uint32_t | 当前连接控制端IP；用于判断是否被其他主机占用或发生抢占。 |
| macAddr0–macAddr5 | uint8_t | MAC地址六字节，合并后形成完整物理地址；用于网络识别与绑定。 |
| temperature0–temperature7 | double | 温度通道0–7（°C），代表内部/外部不同位置；用于热状态监控与告警。 |
| pressure | double | 外部压力（bar），可用于估深度；需校准与水体密度模型以提高精度。 |

## OculusUserConfig

| 字段 | 类型 | 描述 |
|---|---|---|
| ipAddr | uint32_t | 静态IP设置（网络字节序）；用于配置设备地址，需避免与局域网其他设备冲突。 |
| ipMask | uint32_t | 子网掩码设置（网络字节序）；与 ipAddr 一起决定可通信网段范围。 |
| dhcpEnable | uint32_t | DHCP开关，非零表示自动获取IP；开启后静态配置通常会被忽略。 |

## OculusUserConfigMessage

| 字段 | 类型 | 描述 |
|---|---|---|
| head | OculusMessageHeader | 用户配置消息头，标识包类型与长度；用于解析与校验配置写入请求。 |
| config | OculusUserConfig | 网络配置数据（IP/Mask/DHCP），用于写入设备网络参数并重启生效。 |

## OculusReturnFireMessage

| 字段 | 类型 | 描述 |
|---|---|---|
| head | OculusMessageHeader | 返回 Fire 参数消息的头部；用于识别该包为配置回显或诊断数据。 |
| ping | PingConfig | 厂商协议未公开语义的配置块；建议仅记录原始值或透传，避免业务逻辑依赖。 |
| t0–t12 | s0–s12 | 厂商协议未公开语义的参数块集合；建议只记录原始值，便于离线分析与回溯。 |
| ping_params | PingParameters | 含少量注释字段但语义未完整公开；可作调试用途，不建议业务依赖。 |

## PingConfig

| 字段 | 类型 | 描述 |
|---|---|---|
| range | double | 字段名暗示量程相关，但协议未公开；建议仅记录原始值用于调试，不作为算法依据。 |
| nBeams | uint16_t | 字段名暗示 beam 数，但协议未公开；可能与实际 nBeams 不一致，建议仅作参考。 |
| b0, b1–b16, u0–u1, d0, d2–d7 | mixed | 厂商协议未公开语义的占位字段集合；建议仅记录原始值或透传，避免依赖。 |

## s0–s12

| 字段 | 类型 | 描述 |
|---|---|---|
| 全部字段 | mixed | s0–s12 为厂商未公开语义的参数块；字段名为 b*/u*/d*/i* 占位，建议仅记录原始值。 |

## PingParameters

| 字段 | 类型 | 描述 |
|---|---|---|
| imageOffset | uint32_t | 图像数据在包内的字节偏移（注释涉及 CHN/CQI/BQI/BMG 等类型），用于定位图像起点。 |
| imageSize | uint32_t | 图像数据字节长度，用于校验与解析完整性；可作为读包成功的关键验证条件。 |
| messageSize | uint32_t | 完整消息字节长度，用于检测截断或错位；与头部 payloadSize 可互相校验。 |
| nRangeLinesBfm | uint32_t | 字段名暗示 BFM range 行数，但协议未公开；建议仅记录原始值用于调试。 |
| u0–u4, u5–u9, b0–b2, d1–d20 | mixed | 厂商协议未公开语义的占位字段集合；建议只记录原始值或透传，避免依赖。 |

## OculusInfo

| 字段 | 类型 | 描述 |
|---|---|---|
| partNumber | OculusPartNumberType | 型号/部件号枚举；用于匹配设备能力与量程限制。 |
| hasLF | bool | 是否支持低频模式；用于配置层屏蔽不支持的频段，防止无效配置。 |
| maxLF | double | 低频最大量程（米），用于配置上限与安全约束，避免超出设备能力。 |
| hasHF | bool | 是否支持高频模式；用于配置层屏蔽不支持的频段，防止无效配置。 |
| maxHF | double | 高频最大量程（米），用于配置上限与安全约束，避免超出设备能力。 |
| model | char* | 型号描述字符串；用于展示与日志记录，便于现场人员快速识别设备型号。 |
