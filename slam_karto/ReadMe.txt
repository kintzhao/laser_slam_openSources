By Kint Zhao 
========================================2016.1.28   transplant  
  将karto_slam的依赖包open_karto 和  sparse_bundle_adjustment  剥离出来。放到slam_karto中
作为源文件include来调用。。。 

sparse_bundle_adjustment :  BSD 协议
open_karto                  LGPL3


    使这个karto_slam包完全源码。  （除依赖  cxsparse 库）
you should install this  :
 sudo apt-get install  libcsparse3.1.2 libcxsparse3.1.2  libsuitesparse-dev 

=====
open_karto dependent :
  libamd2.3.1 libblas-dev libbtf1.2.0 libcamd2.3.1 libccolamd2.8.0
  libcholmod2.1.2 libcsparse3.1.2 libcxsparse3.1.2 libklu1.2.1 liblapack-dev
  libldl2.1.0 libspqr1.3.1 libumfpack5.6.2
  ros-indigo-open-karto libsuitesparse-dev ros-indigo-sparse-bundle-adjustment

spa  dependent :
  libamd2.3.1 libblas-dev libbtf1.2.0 libcamd2.3.1 libccolamd2.8.0
  libcholmod2.1.2 libcsparse3.1.2 libcxsparse3.1.2 libklu1.2.1 liblapack-dev
  libldl2.1.0 libspqr1.3.1 libsuitesparse-dev libumfpack5.6.2

===
**************libcxsparse3.1.2*****************
/usr/lib/x86_64-linux-gnu/libcxsparse.a  

libcsparse3.1.2 libcxsparse3.1.2  libsuitesparse-dev 


 libsuitesparse-dev depends following :
     libamd2.3.1 libblas-dev libbtf1.2.0 libcamd2.3.1 libccolamd2.8.0
     libcholmod2.1.2 libklu1.2.1 liblapack-dev libldl2.1.0 libspqr1.3.1
     libumfpack5.6.2

=======================================================2016.1.28  raw
the raw karto may occure erros. 
you should :

==============
1.  install the dependens package :  sparse-bundle-adjustment   and open-karto

sudo apt-get install ros-indigo-sparse-bundle-adjustment 
sudo apt-get install ros-indigo-open-karto

===========
2.    change the lanuch files.

problems launching slam_karto (ros-hydro)
http://answers.ros.org/question/201827/problems-launching-slam_karto-ros-hydro/
If you install from source, you need to change the karto_slam.launch in catkin_ws/src/slam_karto/launch: 
The package name is wrong in the launch file 
<node pkg="karto" type="slam_karto" name="slam_karto" output="screen">

Please change the pkg="karto" to pkg="slam_karto",so that the launch file could find the node in slam_karto package.
=============================================
http://blog.csdn.net/hawaecho/article/details/9025975




By Kint Zhao   kint.zhao@slamtec.com
