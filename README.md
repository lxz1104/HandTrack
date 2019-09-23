# 文件说明
`include`文件夹下为头文件，`lib`文件夹下为`.cpp`文件，，`main.cpp`为代码使用的示例，基于**pico flexx**相机和**AXon**相机。
每个文件中都有相应的注释，请仔细阅读

# 环境
以下为我们编写代码并测试所用用到的环境，其他的系统环境并没有经过测试，并不能够保证正常编译和运行
**Linux**
-  **操作系统:** Ubuntu 18.04 amd64
-  **CPU:** Intel i5 8400
-  **GPU:** 此部分代码并没有涉及到使用GPU的部分
-  **开发软件:** Clion 2019.1
-  **编译软件:** cmake 3.15(version >=3.0 ); g++ 7.4.0(能够完全兼容C++11标准的版本即可); make-4.1

**Windows**
-  **操作系统:** Windows10
-  **CPU:** Intel i5 8400
-  **GPU:** 此部分代码并没有涉及到使用GPU的部分
-  **开发软件:** Visual Studio 2019


## 依赖库
- **OpenCV-4.1 + opencv_contrib-4.1.0**(version >= 3.4.6 )

- **Boost-1.65.1**(version >= 1.65.0)

- **libroyale-3.21.1.70-LINUX-x86-64Bit**(深度相机驱动)

- **Eigen3**

# 运行
Windows所有依赖库均在文件夹`3rdParty`，请正确在VS中配置即可(目前所有依赖已配置好，不出意外，直接用VS19打开**.sln**文件即可编译运行，平台一定要选择**X64**。)
!! 为得到较好的效果，请使用**Releases**模式编译运行

