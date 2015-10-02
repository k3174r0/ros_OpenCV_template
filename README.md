ROS OpenCV Template
==
ROS(C++)でOpenCVを使用したノードを作るときにテンプレートになるプログラムです。  
研究室での引き継ぎ用に作りました。ROSでOpenCVプログラムを書く時に参考にしてください。  

cv_bridgeとROSのUSBカメラドライバノードが走っている必要があります。  
**ROS-Indigoで動作します。Hydroでは動くかわかりません。**
## インストール
    sudo apt-get install ros-indigo-cv-bridge
    cd catkin_ws/src
    git clone https://github.com/k3174r0/ros_OpenCV_template.git
    cd ../
    catkin_make
## 使いかた
src/ros_OpenCV_Template.cppのmain関数にOpenCVのコードを書いて実行してください。  
改変する前に、USBカメラの画像を表示するサンプルプログラムが入ってるので実行して確認してください。
## 参考
プログラムでわからないことがあったら参考になるリンク  
* ROS cv_bridge tutorial
http://wiki.ros.org/cv_bridge/Tutorials/UsingCvBridgeToConvertBetweenROSImagesAndOpenCVImages
* OpenCV Document  
http://docs.opencv.org/index.html
