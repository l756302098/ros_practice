# 云台的简单使用
## PELCO-D协议说明  
## 1.云台角度定义  
水平角度范围：0°～360°/-180°～+180°;     垂直角度范围： 0～90°；270°～360°
水平角度定义：水平正前方为0°和360°，面对转动轴向右旋转角度由0°增加，向左旋转角度由360°减少。（俯视向右由0°增加，向左由360°减少）
垂直角度定义：水平正前方为0°和360°，面对转动轴向下旋转角度由0°增加，向上旋转角度由360°减小。
水平和垂直角度的绝对零度：水平正前方。  

## 2.Pelco-D协议格式
Synch Byte：同步字节，始终为 0xFF。  
Byte2：云台地址；0x01-0xFF。  
Byte7：累加和校验。  
   
停止运动命令  
Byte1     	 Byte2   	Byte3	  Byte4	  Byte5	  Byte6	  Byte7  
SynchByte	  Address	  0x00	  0x00	  0x00	  0x00	Check Sum    
  
向上运动命令  
Byte1   	Byte2	  Byte3	Byte4	Byte5	Byte6	Byte7  
SynchByte	Address	0x00	0x08	0x00	Data	Check Sum  
Data表示运行速度，范围是0x00~0x40之间的数值。  



