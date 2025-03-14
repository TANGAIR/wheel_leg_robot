# wheel_leg_robot

# 简介
   本项目为一种单腿三自由度的轮腿复合机器人控制代码。使用Keil MDK进行编程。

1. ```项目机器人展示视频链接1：``` https://www.bilibili.com/video/BV1Gd4y1z7CA/?share_source=copy_web&vd_source=97170e52311d304767c925aed213e556
2. ```项目机器人展示视频链接2：``` https://www.bilibili.com/video/BV1dp4y13755/?share_source=copy_web&vd_source=97170e52311d304767c925aed213e556
3. ```项目机器人展示视频链接3：```  https://www.bilibili.com/video/BV19F411R7DE/?share_source=copy_web&vd_source=97170e52311d304767c925aed213e556
   
 该机器人可以实现轮式运动前进、后退、差速转向以及轮足复合式运动前进、后退、转向，同时具备机身俯仰功能。若配合上位机人脸识别，可实现识别报警功能。
##   项目包含两套代码：

   第1套代码为Big_Dog，这是核心控制代码，基于大疆开发板C型，使用FreeRTOS操作系统进行多任务划分，内容包括机器人运动学解算、机器人位置运动控制、遥控器控制等部分，负责整机控制。
 ```
 ```
   第2套代码为F407电机控制，这是中继代码，基于STM32F407微控制器，其功能是通过串口接收核心控制代码的命令，并将它分发到其他的4个串口，然后通过串口转CAN模块,拓展4路CAN，发送给电机。



## 引用说明

Please cite the following if you use this code or parts of it:

```
@software{tangair2025USB2CAN-Demo-Lingzu,
  author = {tangair},
  title = {{USB2CAN-Demo-Lingzu: An  project based on USB2CAN and Lingzu motor.}},
  url = {https://github.com/SOULDE-Studio/USB2CAN-Demo-Lingzu.git},
  year = {2025}
}
```

