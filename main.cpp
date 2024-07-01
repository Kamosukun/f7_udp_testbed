/*
For NHK2025
NUCLEO-F767ZI + ROS2 Humble
2024/06/30
Comment:
四輪オムニ試作機のMD制御用
ROS2からUDP経由で受け取った数値をMDに出力する
*/

#include "EthernetInterface.h"
#include "mbed.h"
#include "rtos.h"
#include <cstdint>

void receive(UDPSocket *receiver);

DigitalOut MD1D(D4);
PwmOut MD1P(D5);

DigitalOut MD2D(D7);
PwmOut MD2P(D6);

DigitalOut MD3D(D8);
PwmOut MD3P(D9);

DigitalOut MD4D(D12);
PwmOut MD4P(D10);

DigitalOut MD5D(D13);
PwmOut MD5P(D11);

double mdd[6];
double mdp[6];

int main() {
  /*PWM周波数の設定
  us = μs
  period_us = 1000(ms) / MDのPWM周波数(Hz) * 10^3
  ex: 50(us) = 1000(ms) / (20(kHz) * 10^3) * 10^3
  */
  MD1P.period_us(50);
  MD2P.period_us(50);
  MD3P.period_us(50);
  MD4P.period_us(50);
  MD5P.period_us(50);

  // 送信先情報
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
    printf("Network connection Error\n");
    return -1;
  } else {
    printf("Network connection success\n");
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
  SocketAddress source;
  char buffer[64];

  int data[6] = {0, 0, 0, 0, 0, 0};
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
      // 0.0~1.0の範囲にマッピング
      /*
      printf("%d, %d, %d, %d, %d\n", data[1], data[2], data[3], data[4],
             data[5]);*/

      for (int i = 1; i <= 5; i++) {
        if (data[i] >= 0) {
          mdd[i] = 1;
        } else {
          mdd[i] = 0;
        }
        mdp[i] = fabs(data[i]) / 255;
      }

      printf("%f, %f, %f, %f, %f\n", mdp[1], mdp[2], mdp[3], mdp[4], mdp[5]);

      ///////////////////////////////////////////////////////////////////////////////////
      // Output

      MD1D = mdd[1];
      MD2D = mdd[2];
      MD3D = mdd[3];
      MD4D = mdd[4];
      MD5D = mdd[5];

      MD1P = mdp[1];
      MD2P = mdp[2];
      MD3P = mdp[3];
      MD4P = mdp[4];
      MD5P = mdp[5];

      ///////////////////////////////////////////////////////////////////////////////////
    }
  }
}