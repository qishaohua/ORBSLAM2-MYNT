# 基于光流法的动态语义地图构建

# 语义地图综述
针对地图中实例的分割
[基于视觉SLAM的物体实例识别与语义地图构建](https://kns.cnki.net/KCMS/detail/detail.aspx?dbcode=CJFQ&dbname=CJFDTEMP&filename=HZLG201909009&uid=WEEvREcwSlJHSldRa1FhdXNXaEhobnVuRkVVTkdQaENycDFtMEFKdW05RT0=$9A4hF_YAuvQ5obgVAqNKPCYcEjKensW4IQMovwHtwkF4VYPoHbKxJw!!&v=MzE3NzMzcVRyV00xRnJDVVJMT2VaK1J1RmlEaFVMM1BMVGZIYWJHNEg5ak1wbzlGYllSOGVYMUx1eFlTN0RoMVQ=)


# 位姿估计
[刚体6D姿态估计方法综述](https://blog.csdn.net/dsoftware/article/details/97955570)  


# ORBSLAM2-MYNT

末尾两个空格实现换行  
[protobuf简单介绍和ubuntu 16.04环境下安装](https://blog.csdn.net/kdchxue/article/details/81046192)   

[vector的初始化方式](https://blog.csdn.net/qq_28584889/article/details/83654318)  

[ncnn安装](https://github.com/Tencent/ncnn/wiki/how-to-build#build-for-linux-x86)  

[ncnn使用](https://github.com/Ewenwan/MVision/tree/master/CNN/HighPerformanceComputing/example)  

[caffe安装](https://www.cnblogs.com/xuanxufeng/p/6150593.html)  

[caffe-ssd](https://blog.csdn.net/lukaslong/article/details/81390276)  

[SSD的配置安装与测试](https://blog.csdn.net/shawncheer/article/details/53227212/)  

[caffe安装报错](https://www.cnblogs.com/haiyang21/p/10214278.html)  

[ImportError: dynamic module does not define module export function (PyInit__caffe)](https://blog.csdn.net/qq_35759574/article/details/83750766)  

[ImportError: No module named _caffe](https://blog.csdn.net/weixin_41803041/article/details/79495617)  

[protoc: error while loading shared libraries: libprotoc.so.10:](https://yq.aliyun.com/articles/41611)

[Ubuntu16.04下完美切换Python版本](https://blog.csdn.net/beijiu5854/article/details/77897767)

别忘记在Makefile.config中修改算力
[caffe报错 libcudnn.so.7](https://blog.csdn.net/sinat_23619409/article/details/85047788)  

[protobuf-2.5.0.tar.gz的下载与安装](https://www.tuicool.com/articles/jM7Nn2/)

[caffe Cmake安装报错1](https://blog.csdn.net/qq_42189368/article/details/87252919)  

[caffe  python3编译caffe错误:cannot find -lboost_python3](https://blog.csdn.net/songyu0120/article/details/77895373)

[Ubuntu下，清屏等终端常用命令](https://blog.csdn.net/gaojinshan/article/details/9314435)  

[ubuntu 16.04安装 python3 caffe](https://blog.csdn.net/sinat_27240143/article/details/82701285)  

[编译过程中的一些问题](https://github.com/Ewenwan/ORB_SLAM2_SSD_Semantic/issues/2)  

'dict_keys' object has no attribute 'remove' 解决方案：  
#first_keys = first_list.keys()  
#second_keys = second_list.keys()  
first_keys = list(first_list)  
second_keys = list(second_list)  

[Ubuntu16.04+cuda9.0安装教程](https://www.cnblogs.com/iloveblog/p/7683349.html)  
[CUDA9.1、cuDNN7在Ubuntu16.04上的安装](https://blog.csdn.net/jonms/article/details/79318566)  
[OpenCV - Linux（Ubuntu 16.04）中安装OpenCV + OpenCV_Contrib](https://www.cnblogs.com/fx-blog/p/8213704.html)  
[ubuntu16.04安装opencv3.4](https://blog.csdn.net/u013066730/article/details/79411767)  
[ubuntu 安装Pangolin 过程](https://blog.csdn.net/u012986684/article/details/52860849)  
[ORB-SLAM2 with Octomap](https://blog.csdn.net/qq_27840681/article/details/80168678)  
  
[Ubuntu 16.04安装tensorflow_gpu 1.9.0的方法](https://blog.csdn.net/davidhopper/article/details/81206673)  

[图漾相机安装 ROS](https://blog.csdn.net/learning_tortosie/article/details/80887896)  
注意： sudo apt-get install ros-indigo-rgbd-launch 这句是Ubuntu14.04版本的， 在16.04下用 sudo apt-get install ros-kenetic-rgbd-launch
      
软件卸载  
sudo rm -r /usr/include/pcl-1.7 /usr/share/pcl /usr/bin/pcl* /usr/lib/libpcl*  
sudo apt-get purge libvtk*  
  
  
  

YOLO运行  
./darknet detector demo cfg/coco.data cfg/yolov3.cfg yolov3.weights  
./darknet detect cfg/yolov3.cfg yolov3.weights data/eagle.jpg  
  
    
      
[Ubuntu16.04编译高博的ORBSLAM2_with_pointcloud_map](https://blog.csdn.net/qq_25349629/article/details/88350374)  

  
[ubuntu16配置Mask-RCNN](https://www.cnblogs.com/herd/p/9364911.html)  
[[论文笔记]Meaningful Maps With Object-Oriented Semantic Mapping](https://blog.csdn.net/pikachu_777/article/details/84570980)  

[最新网址get](http://zlibz.com/)  

  
[图漾 Percipio FM810-HD 的使用](https://blog.csdn.net/learning_tortosie/article/details/80887896)  

[ORB-SLAM2主页](https://github.com/raulmur/ORB_SLAM2)  

[MYNT-SDK](https://slightech.github.io/MYNT-EYE-D-SDK/build_linux.html)  

[ubuntu16安装ROS](https://blog.csdn.net/tq08g2z/article/details/79209435)  

## opencv3.4.0安装  

[opencv_contrib网址](https://github.com/opencv/opencv_contrib)  

[Ubuntu16.04卸载opencv 3.0.0，安装opencv3.4.2 + contrib……1](https://blog.csdn.net/haoqimao_hard/article/details/82049565)  
[Ubuntu16.04卸载opencv 3.0.0，安装opencv3.4.2 + contrib……2](https://blog.csdn.net/cocoaqin/article/details/78163171)  


pkg-config opencv --libs  
pkg-config opencv --modversion  
sudo find / -iname "*opencv*"  
find . -name "*opencv*" | xargs sudo rm -rf  

cmake -D CMAKE_BUILD_TYPE=Release -D CMAKE_INSTALL_PREFIX=/usr/local -D OPENCV_EXTRA_MODULES_PATH=~/Package/opencv3.4.0/opencv_contrib/modules/ ..  

[ubuntu下查看opencv安装路径以及版本号](https://blog.csdn.net/xunan003/article/details/82144924)

## YOLO GPU配置
（一）.修改makefile 文件  

            1.打开darknet/MakeFile文件，将GPU=0 --> GPU=1

            2.CUDNN=0 --> CUDNN=1

            3.DEBUG=0 --> DEBUG=1

           4.NVCC=/usr/local/cuda/cuda/bin/nvcc

               -->NVCC=/usr/local/cuda-8.0/bin/nvcc（这里nvcc所在位置注意修改成自己的）

           5.COMMON+= -DGPU -I/usr/local/cuda/include

              -->COMMON+= -DGPU -I/usr/local/cuda-8.0/include

           6.LDFLAGS+= -L/usr/local/cuda/lib64-lcuda -lcudart -lcublas -lcarand

              --> LDFLAGS+= -L/usr/local/cuda-8.0/lib64-lcuda -lcudart -lcublas -lcarand



  (二). 解决cudnn.h问题，重新下载加入文件夹中并链接
 cudnn解压文件后会看到一个cuda文件夹，里面包含了include以及lib64两个子目录。

             将这两个文件夹里的文件复制到cuda对应的安装目录。这里以cuda的安装目录为/usr/local/cuda/为例：

               sudo cp ~/slam/package/cuda/include/cudnn.h /usr/local/cuda/include


              sudo cp ~/slam/package/cuda/lib64/* /usr/local/cuda/lib64

链接

            #下面的操作在/usr/local/cuda/lib64/目录下进行

               sudo rm -rf libcudnn.so libcudnn.so.7.0#删除两个符号链接；

               sudo ln -s libcudnn.so.7.0.64 libcudnn.so.7.0

               sudo ln -s libcudnn.so.7.0 libcudnn.so




（三）. 解决找不到lcuda等库问题，查找并连接  

               /usr/bin/ld: cannot find -lcarand

               /usr/bin/ld: cannot find -lcuda

               /usr/bin/ld: cannot find -lcudnn

            注：lcuda 意思是libcuda，用如下命令定位libcuda位置  



## yolov3编译问题

yolov3 CUDA Error: out of memory darknet: ./src/cuda.c:36: check_error: Assertion `0' failed.

https://blog.csdn.net/fanhongyuan21/article/details/81909994

将yolov3.cfg中的

    # Testing
    batch=1       取消注释
    subdivisions=1
    # Training
    #batch=64     注释掉
    #subdivisions=16

错是因为其申请了太多的内存而内存不够

## ORBSLAM2编译ncnn  
../lib/libORB_SLAM2_pc.so: undefined reference to `ncnn::Layer::destroy_pipeline(ncnn::Option const&)'  
../lib/libORB_SLAM2_pc.so: undefined reference to `ncnn::Layer::create_pipeline(ncnn::Option const&)'  
collect2: error: ld returned 1 exit status  

解决方案：
ncnn:  make install --- 把 build/install/lib/libncnn.a 关联到 Cmake 文件中  
${PROJECT_SOURCE_DIR}/Thirdparty/ncnn/build/install/lib/libncnn.a  

## QObject::killTimer: Timers cannot be stopped from another thread
QObject::startTimer: Timers cannot be started from another thread  
解决方案：




# common
PC常见问题


[win10+ubuntu彻底删除ubuntu](https://www.jianshu.com/p/42af78f88962)  
然后打开EasyUEFI，删除Ubuntu启动项  

### 经费卡密码 8888


### 图漾相机问题解决  
问题描述：运行应用程序后，如果提示”Couldn’t open device, some information will be missing, ret: -3”, 请检查系统的访问权限配置是否正确。  
sudo ./my_rgbd ../../Vocabulary/ORBvoc.bin my_rgbd_ty_api_adj.yaml


### 卸载OpenCV的方法：进入OpenCV解压文件夹中的buid 文件夹：

cd opencv-3.4.0/build

运行：

sudo make uninstall

然后把整个opencv-3.4.0文件夹都删掉。随后再运行：

sudo rm -r /usr/local/include/opencv2 /usr/local/include/opencv /usr/include/opencv /usr/include/opencv2 /usr/local/share/opencv /usr/local/share/OpenCV /usr/share/opencv /usr/share/OpenCV /usr/local/bin/opencv* /usr/local/lib/libopencv



### ORBSLAM2_OCTOMAP_ROS
roscore

cd ~/Package/camport_ros/  
roslaunch openni2_launch openni2.launch

roscd ORB_SLAM2/  
rosrun ORB_SLAM2 RGBD ../../../Vocabulary/ORBvoc.bin my_rgbd_ty_api_adj.yaml 

### ORBSLAM2_PCL_ROS
roscd ORB_SLAM2-pcl/  
rosrun ORB_SLAM2-pcl RGBD ../../../Vocabulary/ORBvoc.bin my_rgbd_ty_api_adj.yaml 


[ROS实现两台计算机之间的网络通信](https://www.cnblogs.com/liu-fa/p/5773822.html)  

[电脑蓝屏](https://mp.weixin.qq.com/s?__biz=MzUyNzc0ODI1Nw==&mid=2247484202&idx=1&sn=299be09386fb9c90d7efea4f106dd45c&chksm=fa7b9ee4cd0c17f2764dd5df1fbf43bb8c6ddcd8e836d368d6892e93a623e41709772c917d77&mpshare=1&scene=1&srcid=12244sCMAMM2CqbIYSZjUubw#rd)  


[ROS 编译错误以及解决方案](https://blog.csdn.net/u011608180/article/details/88419732)
