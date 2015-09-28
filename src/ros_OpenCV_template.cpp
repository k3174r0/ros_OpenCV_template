/*ROSでUSBカメラを使用してOpenCVによる処理をするプログラムのひな形*/

/*
ROS Wikiのcv_bridgeチュートリアルサンプルを参考に、メイン関数とメインループが見やすい形で
OpenCVのプログラムを書き始められるように、テンプレートプログラムを書きました。
*/

/*必要なヘッダーを追加*/
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

/*グローバル変数としてcv_bridge::CvImagePtr型のcv_ptrという名前の変数を宣言 cv_ptrにOpenCVで扱う画像データが格納される*/
cv_bridge::CvImagePtr cv_ptr;

/*画像データをSubscribeすることによって呼ばれるコールバック関数*/
void imageCb(const sensor_msgs::ImageConstPtr& msg)
{
/*tryとcatchは例外処理に用いられる*/

/*例外が発生する可能性のあるコードをtryで囲む*/
  try
  {
/*画像のデータをROSとOpenCVが扱えるようにしてcv_ptrに入れる*/
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);//例外が発生する可能性があるコード
  }

/*catchで囲まれたコードはtry内で例外が発生した時に実行される catchはtryの直後に記述する*/
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

}

/*main関数*/
int main(int argc, char** argv)
{
/*ノード初期化*/
  ros::init(argc, argv, "OpenCV_template");

/*ノードハンドルの生成＆初期化*/
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);

/*Publisher・Subscriberの設定*/
/*image_transport::Subscriberで設定するトピック名は、使用するカメラドライバノードによってSubscribeされるメッセージに合わせる必要がある。
  Wikiやネットで調べるか、カメラドライバノードをrosrunしてからrostopic listかrqt_image_viewで確認して名前を見つける
　使用するカメラドライバノードによってトピック名を適宜変える	      ↓/image_rawでは無い場合がある。　この例ではuvc_camera_nodeを使用*/
  image_transport::Subscriber image_sub_ = it.subscribe("/image_raw", 1, imageCb);
  image_transport::Publisher image_pub_ = it.advertise("/image_converter/output_video", 1);

/*OpenCVで生成するウィンドウに名前をつける*/
  cv::namedWindow("Image Window");

/*メインループ*/
  while(ros::ok()){  //ros::ok()は基本的にtrueを返すのでこの中のコードが繰り返されるが^C、同名ノード、接続断の際にfalseを返す。

/*cv_ptrの中身がある場合の処理 
  cv_ptrはimageCb関数で中身が更新されていくが一番最初メッセージがSubscribeされる前や何らかのエラーによる不具合を防ぐ*/
    if(cv_ptr){

/*------------------------------------------------------*/
/*		ここにOpenCVのコードを書く			*/
/*		USBカメラからの画像を処理するときは		*/
/*		cv_ptr->imageを操作してください		*/
/*	cv::Mat型の画像データがcv_ptr->imageにあります	*/
/*							*/
/*	　　その他変数等は宣言する必要があります		*/
/*　　OpenCVプログラムに必要な最低限のヘッダーは追加済みです	*/
/*							*/
/* 下にあるコードはWebカメラの画像をそのまま表示するコードの例	*/
/* ↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓	*/
/*------------------------------------------------------*/

/*OpenCVでウインドウを生成しcv_ptr内のimageデータを表示する*/  
      cv::imshow("Image Window", cv_ptr->image);
/*【重要】何故かcv::waitkey()が無いとビルドできてもノードが正常に走らない 
  cv::waitkey(int)はキー入力を検出する関数だが動作には関係がない？　cv::waitkey(n):nは(int)数字、単位はms*/
      cv::waitKey(1);
    }
/*コールバックを拾うための関数 Subscribeするノードの場合必ず必要*/
    ros::spinOnce();
  }
}
