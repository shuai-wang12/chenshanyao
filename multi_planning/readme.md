调试需要 
catkin_make -DCMAKE_BUILD_TYPE=Debug

多车的话是随便选起点终点就会开始运行，在程序里设置的起点终点
所以在rviz中设置起点终点只是开始运行的标志

程序是在单车规划的结果上进行Cbs，就是输入是单车规划的csv文件，这个程序
只有cbs算法，当然可以改成统一整合，我已经整好的，就是赶时间才改成这样做
所以核心文件就是main.cpp planner.cpp multi_opti.cpp cl_cbs.cpp

单车规划是在另一个工程中

有问题: Vx❤ a1xiaobaicai

参考代码
https://github.com/APRIL-ZJU/CL-CBS.git