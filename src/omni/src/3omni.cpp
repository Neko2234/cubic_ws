#include <ros/ros.h>
#include <std_msgs/Int16MultiArray.h>
#include <sensor_msgs/Joy.h>
#include <string>
#include <time.h>

#define MOTOR_NUM 3
#define PW_MAX 100 //モータ出力絶対値の最大値
#define SQRT2 1.41421356
#define SQRT3 1.7320504
#define DIRE 1 //モータ回転方向調整用の変数

std::string mode = "progress";
std_msgs::Int16MultiArray order;

ros::Publisher pub;
ros::Subscriber sub;
ros::Subscriber enc_sub;

double vx = 0, vy = 0, ang = 0;
double v_raw[MOTOR_NUM] = {0};
int16_t v[MOTOR_NUM] = {0};
int e_raw[MOTOR_NUM] = {0};
double t;
double s_raw[MOTOR_NUM] = {0};
int16_t s[MOTOR_NUM] = {0};

void callback(const sensor_msgs::Joy::ConstPtr& msg) {
    //-1.0 <= vx,vy <= 1.0
    vx = -msg->axes[0], vy = msg->axes[1], ang = -msg->axes[2];
}

void encCb(const std_msgs::Int16MultiArray& enc_msg){
	e_raw[0] = enc_msg.data[0];
	e_raw[1] = enc_msg.data[1];
	e_raw[2] = enc_msg.data[2];
}

void target_culc(){
	for(int i=0; i<MOTOR_NUM; i++){
		s_raw[i] = e_raw[i]/t;//時間あたりのパルス
	}
	
    int s_sum=1;
    s_sum = (s_raw[0] + s_raw[1] + s_raw[2]);
    
    if(s_sum != 0){
        for(int i; i<MOTOR_NUM; i++){
	    	s[i] = (int)((s_raw[i] / s_sum)*100);
        }
    }else if(s_sum == 0){//エラー
        s[0] = s_raw[0];
        s[1] = 39;
        s[2] = 39;
    }
}

void omni_calc() {
    if (mode == "progress") {
        //位置:回転=1:sqrt2 → 1:1 にスケーリング
        //ang *= 0.5*SQRT2;

        v_raw[0] =         vx                + ang;
        v_raw[1] =   - 0.5*vx - 0.5*SQRT3*vy + ang;
        v_raw[2] =   - 0.5*vx + 0.5*SQRT3*vy + ang;

        /*v_raw[0] = - vx - vy + ang;
        v_raw[1] =   vx - vy + ang;
        v_raw[2] =   vx + vy + ang;
        v_raw[3] = - vx + vy + ang;*/

        //PW_MAX
        for (int i = 0; i < MOTOR_NUM; i++) {
            v[i] = v_raw[i] * PW_MAX / 3.0;
        }
    }
}

void publish() {
    //order.data.resize(4);
    for (int i = 0; i < MOTOR_NUM; i++) {
	order.data[i] = s[i];
        //order.data[i] = v[i];
    }
    pub.publish(order);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "omni");
    ros::NodeHandle nh;

    order.data.assign(MOTOR_NUM,0);

    pub = nh.advertise<std_msgs::Int16MultiArray>("omni", 1);
    sub = nh.subscribe("joy", 1, callback);
    enc_sub = nh.subscribe("encoder", 1, encCb);

    ros::Rate loop_rate(10);
    while(ros::ok()) {
    	clock_t start = clock();    // スタート時間
        ros::spinOnce();
	
	target_culc();
        omni_calc();

        publish();

        loop_rate.sleep();
        clock_t end = clock();     // 終了時間
        t = (double)start - end;
    }
}
