/*
Framework for UDP MD Driver on F7
2024/06/30
*/

#include "EthernetInterface.h"
#include "mbed.h"
#include "rtos.h"
#include <cstdint>

void receive(UDPSocket *receiver);
void output_process();

typedef struct {
  int one;
  int two;
  int threee;
  int four;
  int five;
} message_t;

MemoryPool<message_t, 24> mpool;
Queue<message_t, 24> queue;
Thread thread;

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

  Thread outputThread;

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

  outputThread.start(callback(output_process));
  outputThread.join();

  int data[6] = {0, 0, 0, 0, 0, 0};

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
        ///////////////////////////////////////////////////////////////////////////////////
      }

      message_t *message = mpool.try_alloc();
      message->one = data[1];
      message->two = data[2];
      message->threee = data[3];
      message->four = data[4];
      message->five = data[5];

      queue.try_put(message);
    }
  }
}

void output_process() {

  while (1) {
    osEvent evt = queue.get();
    if (evt.status == osEventMessage) {
      message_t *message = (message_t *)evt.value.p;
      data[1] = message->one;
      data[2] = message->two;
      data[3] = message->threee;
      data[4] = message->four;
      data[5] = message->five;
      mpool.free(message);
    }
    ///////////////////////////////////////////////////////////////////////////////////
    // 0.0~1.0の範囲にマッピング
    printf("%d, %d, %d, %d, %d\n", data[1], data[2], data[3], data[4], data[5]);

    for (int i = 1; i <= 5; i++) {
      if (data[i] >= 0) {
        mdd[i] = 1;
      } else {
        mdd[i] = 0;
      }
      mdp[i] = fabs(data[i]) / 255;
    }

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