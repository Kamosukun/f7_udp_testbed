/*
4輪オムニ試作機
ROS2から速度指令をRPMで受信
エンコーダーからRPMを求めPID制御をかける　
F7メイン基板V2向けにピン割り当てを変更
2024/07/22
*/

#include "EthernetInterface.h"
#include "QEI.h"
#include "mbed.h"
#include "rtos.h"
#include <cstdint>

/// QEI
QEI ENC1(PC_0, PG_1, NC, 2048, QEI::X4_ENCODING);
QEI ENC2(PF_2, PC_3, NC, 2048, QEI::X4_ENCODING);
QEI ENC3(PD_4, PF_5, NC, 2048, QEI::X4_ENCODING);
QEI ENC4(PA_6, PF_7, NC, 2048, QEI::X4_ENCODING);
QEI ENC5(PE_8, PF_9, NC, 2048, QEI::X4_ENCODING);
QEI ENC6(PF_10, PD_11, NC, 2048, QEI::X4_ENCODING);

/*
QEI (A_ch, B_ch, index, int pulsesPerRev, QEI::X2_ENCODING)
index -> Xピン, １回転ごとに１パルス出力される？ 使わない場合はNCでok
pulsePerRev -> Resolution (PPR)を指す
X4も可,X4のほうが細かく取れる

データシート(

): https://jp.cuidevices.com/product/resource/amt10-v.pdf
*/
// end

using ThisThread::sleep_for;

void receive(UDPSocket *receiver);

PwmOut MD1P(PA_0);
PwmOut MD2P(PA_3);
PwmOut MD3P(PB_4);
PwmOut MD4P(PB_5);
PwmOut MD5P(PC_7);
PwmOut MD6P(PC_6);
PwmOut MD7P(PC_8);
PwmOut MD8P(PC_9);

DigitalOut MD1D(PD_2);
DigitalOut MD2D(PG_2);
DigitalOut MD3D(PG_3);
DigitalOut MD4D(PE_4);
DigitalOut MD5D(PD_5);
DigitalOut MD6D(PD_6);
DigitalOut MD7D(PD_7);
DigitalOut MD8D(PC_10);

PwmOut SERVO1(PB_1);
PwmOut SERVO2(PB_6);
PwmOut SERVO3(PD_13);
PwmOut SERVO4(PD_12);

int Pulse[7];
int last_Pulse[7];
int dt = 0;
double dt_d = 0; // casted dt
double RPM[7];

double target[7];
double Kp;
double Ki;
double Kd;
double Error[7];
double last_Error[7];
double Integral[7];
double Differential[7];
double Output[7];
double rpm_limit;
double pwm_limit; // PWM出力制限　絶対に消すな

double mdd[9];
double mdp[9];

int main() {

  // PWM Setting
  MD1P.period_us(50);
  MD2P.period_us(50);
  MD3P.period_us(50);
  MD4P.period_us(50);
  MD5P.period_us(50);
  MD6P.period_us(50);
  MD7P.period_us(50);
  MD8P.period_us(50);
  /*
  50(us) = 1000(ms) / 20000(Hz)
  MDに合わせて調整
  CytronのMDはPWM周波数が20kHzなので上式になる
  */
  // end

  // 送信先情報(F7)
  const char *destinationIP = "192.168.8.205";
  const uint16_t destinationPort = 4000;

  // 自機情報
  const char *myIP = "192.168.8.215";
  const char *myNetMask = "255.255.255.0";
  const uint16_t receivePort = 5000;

  // イーサネット経由でインターネットに接続するクラス
  EthernetInterface net;
  // IPアドレスとPortの組み合わせを格納しておくクラス（構造体でいいのでは？）
  SocketAddress destination, source, myData;
  // UDP通信関係のクラス
  UDPSocket udp;
  // 受信用スレッド
  Thread receiveThread;

  /* マイコンのネットワーク設定 */
  // DHCPはオフにする（静的にIPなどを設定するため）
  net.set_dhcp(false);
  // IPなど設定
  net.set_network(myIP, myNetMask, "");

  printf("Start\n");

  // マイコンをネットワークに接続
  if (net.connect() != 0) {
    printf("Network connection Error (>_<)\n");
    return -1;
  } else {
    printf("Network connection success (^_^)\n");
  }

  // UDPソケットをオープン
  udp.open(&net);

  // portをバインドする
  udp.bind(receivePort);

  // 送信先の情報を入力
  destination.set_ip_address(destinationIP);
  destination.set_port(destinationPort);
  // 受信用のスレッドをスタート
  receiveThread.start(callback(receive, &udp));

  receiveThread.join();

  udp.close();
  net.disconnect();
  return 0;
}

void receive(UDPSocket *receiver) {

  using namespace std::chrono;

  Timer t;
  t.start();
  SocketAddress source;
  char buffer[64];

  int data[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0};

  // PID parameter
  Kp = 0.1;
  Ki = 0.0015;
  Kd = 0.000000001;
  rpm_limit = 60.0;
  pwm_limit = 0.6;
  //動作に影響するようなら#defineへ
  // end

  while (1) {
    memset(buffer, 0, sizeof(buffer));
    if (const int result =
            receiver->recvfrom(&source, buffer, sizeof(buffer)) < 0) {
      printf("Receive Error : %d", result);
    } else {

      ///////////////////////////////////////////////////////////////////////////////////
      // 受信したパケット（文字列）を処理
      char *ptr;
      int ptr_counter = 1;
      // カンマを区切りに文字列を分割
      // 1回目
      ptr = strtok(buffer, ",");
      data[1] = atoi(ptr); // intに変換

      // 2回目以降
      while (ptr != NULL) {
        ptr_counter++;
        // strtok関数により変更されたNULLのポインタが先頭
        ptr = strtok(NULL, ",");
        data[ptr_counter] = atoi(ptr); // intに変換

        // ptrがNULLの場合エラーが発生するので対処
        if (ptr != NULL) {
          // printf("%s\n", ptr);
        }
      }
      ///////////////////////////////////////////////////////////////////////////////////
      //方向指令と速度指令を分離する
      /*printf("%d, %d, %d, %d, %d\n", data[1], data[2], data[3], data[4],
             data[5]);*/

      for (int i = 1; i <= 5; i++) {
        if (data[i] > 0) {
          mdd[i] = 1;
        } else if (data[i] < 0) {
          mdd[i] = 0;
        }
        // 0.0~1.0の範囲にマッピング
        // mdp[i] = fabs(data[i]) / 255;
      }
      t.stop();

      //エンコーダーから値を取得&RPMの計算//////////////////////////////////////////////////////////////
      Pulse[1] = ENC1.getPulses();
      Pulse[2] = ENC2.getPulses();
      Pulse[3] = ENC3.getPulses();
      Pulse[4] = ENC4.getPulses();
      Pulse[5] = ENC5.getPulses();
      Pulse[6] = ENC6.getPulses();
      dt = duration_cast<milliseconds>(t.elapsed_time()).count();

      for (int i = 1; i <= 6; i++) {
        RPM[i] = 60000.0 / dt * (Pulse[i]) /
                 8192; // 現在のRPM（1分間当たりの回転数）を求める
        // printf("%d\n", RPM);
        RPM[i] = fabs(RPM[i]);
      }
      ENC1.reset();
      ENC2.reset();
      ENC3.reset();
      ENC4.reset();
      ENC5.reset();
      ENC6.reset();
      /////////////////////////////////////////////////////////////////////////////////////////////

      // PID///////////////////////////////////////////////////////////////////////////////////////

      dt_d = (double)dt / 1000000000; // cast to double
      for (int i = 1; i <= 6; i++) {
        target[i] = abs((double)data[i]) / rpm_limit;
        Error[i] = target[i] - (RPM[i] / rpm_limit);            // P
        Integral[i] += ((Error[i] + last_Error[i]) * dt_d / 2); // I
        Differential[i] = (Error[i] - last_Error[i]) / dt_d;    // D

        last_Error[i] = Error[i];
        Output[i] += ((Kp * Error[i]) + (Ki * Integral[i]) +
                      (Kd * Differential[i])); // PID
        mdp[i] = Output[i];

        // 安全のためPWMの出力を制限　絶対に消すな
        if (mdp[i] > pwm_limit) {
          mdp[i] = pwm_limit;
        } else if (mdp[1] < 0.0) {
          mdp[i] = 0.0;
        }
        // end
      }

      t.reset();
      t.start();
      /*
      printf("%lf, %lf, %lf, %lf, %lf\n", mdp[1], mdp[2], mdp[3], mdp[4],
             mdp[5]);
             */
      // モーターがうまく回らないときは要調整、短すぎるとPIDがうまく動かず、長すぎるとレスポンスが悪くなる
      sleep_for(50);

      ////////////////////////////////////////////////////////////////////////////////////////////

      // Output////////////////////////////////////////////////////////////////////////////////////

      MD1D = mdd[1];
      MD2D = mdd[2];
      MD3D = mdd[3];
      MD4D = mdd[4];
      MD5D = mdd[5];
      MD6D = mdd[6];
      MD7D = mdd[7];
      MD8D = mdd[8];

      MD1P = mdp[1];
      MD2P = mdp[2];
      MD3P = mdp[3];
      MD4P = mdp[4];
      MD5P = mdp[5];
      MD6P = mdp[6];
      MD7P = mdp[7];
      MD8P = mdp[8];

      ///////////////////////////////////////////////////////////////////////////////////
    }
  }
}