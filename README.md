## 2024第十九届全国大学生智能汽车竞赛 完全模型组 全国亚军代码开源

### 上位机 upper_com

* 基于'赛曙开源代码'修改，根据赛曙指南重新编译后，可直接运行。
* 亮点

> * 利用future库实现多线程，简单，同时效果极佳，平均帧率由30提升至150
> * 基于原图像进行元素识别，仅对搜索到的边线逆透视，用以拟合中线，减少计算量
> * 纯跟随结合pd实现舵机控制，避免了舵机-角度关系的采集；同时对p、d参数的依赖性较弱，鲁棒性较好
> * 简单的拟合实现动态pd、速度策略

### 下位机 lower_com

* 基于'逐飞TC264c开源库'开发，使用'AURIX™ Development Studio'编译。
* 仅进行舵机与电机控制、与上位机通信
* 通信代码中，部分代码针对于救援区与上位机进行特定交互                                                                                                                                                                                                                                                                                               



### 比赛记录

* 华南省赛

<img src=".\华南省赛.gif" alt="华南省赛" width="500" height="auto">



* 国赛

<img src=".\国赛.gif" alt="国赛" width="500" height="auto">



### 致谢

​	感谢在一年智能车生涯中帮助我的学长们，也感谢我的学长兼队友越神和乾哥一路以来的陪伴，以及对我冲向国特愿望的鼎力支持。

​	或是由于自己临场紧张导致决赛时间的白白浪费，或是因为自己平时准备的不充分和侥幸，最后的结果有些许遗憾，也算是感受到了智能车的魅力。

​	无论如何，这是我人生以来一段难以忘却的美好回忆，也希望这份代码能够帮助更多人去更好地感受智能车比赛，将这份美好传递下去。
