# Vins Course
**作者**：贺一家，高翔，崔华坤，赵松

**描述**：
这是一个用于深蓝学院教学的代码，她基于 VINS-Mono 框架，但不依赖 ROS, Ceres, G2o。这个代码非常基础，目的在于演示仅基于 Eigen 的后端 LM 算法，滑动窗口算法，鲁棒核函数等等 SLAM 优化中常见的算法。
该代码支持 Ubuntu or Mac OS.

### 安装依赖项：

1. pangolin: <https://github.com/stevenlovegrove/Pangolin>

2. opencv

3. Eigen

4. Ceres: vins 初始化部分使用了 ceres 做 sfm，所以我们还是需要依赖 ceres. 

### 编译代码

```c++
mkdir vins_course
cd vins_course
git clone https://github.com/HeYijia/VINS-Course
mkdir build 
cd build
cmake ..
make -j4
```

### 运行
#### 1. CurveFitting Example to Verify Our Solver.
```c++
cd build
../bin/testCurveFitting 
```

#### 2. VINs-Mono on Euroc Dataset
```c++
cd build
../bin/run_euroc /home/dataset/EuRoC/MH-05/mav0/ ../config/
```
![vins](doc/vins.gif)

#### 3. VINs-Mono on Simulation Dataset (project homework)

you can use this code to generate vio data.

```c++
https://github.com/HeYijia/vio_data_simulation
```

### Licence

The source code is released under GPLv3 license.

We are still working on improving the code reliability. For any technical issues, please contact Yijia He <heyijia_2013@163.com> , Xiang Gao <https://github.com/gaoxiang12> or Huakun Cui<https://github.com/StevenCui>.

For commercial inquiries, please contact Song Zhao <zhaosong@shenlanxueyuan.com>

### 感谢

我们使用了港科大沈老师组的 [VINS-Mono](https://github.com/HKUST-Aerial-Robotics/VINS-Mono) 作为基础代码，非常感谢该组的工作。

大作业为两个代码作业, 完成时间为三周:     

1. 更优的优化策略,:   
   
   a. 选用更优的 LM 策略, 使得 VINS-Mono 在 MH-05 数据集上收敛速度更快或者精度更高.  
   
   b. 实现 dog-leg 算法替换 LM 算法, 并测试替换后的 VINS-Mono 在 MH-05 上算法精度.    
   
   详细的实验报告,包括: 对迭代时间和精度进行评估, 其中精度评估可以采用 evo 工具( https://github.com/MichaelGrupp/evo ) 对轨迹精度进行评估, 轨迹真值在 zip 中已给出.   

   

2. 更快的 makehessian 矩阵  

   可以采用任何一种或多种加速方式 (如多线程, 如sse指令集等) 对信息矩阵的拼接函数加速, 并给出详细的实验对比报告.
