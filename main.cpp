#include <mbed.h>
#include <iostream>
#include <cmath>

// ラジアンを度に変換するための定数
const double RAD_TO_DEG = 180.0 / M_PI;
uint8_t DATA[8] = {};

/*
三輪オムニの出力[1,2,3]
　　前斜左-----前斜右
　　　|　　　　　|
　　　　|　　　|
　　　　　|　|
　　　　　後ろ

前斜左:1   Θ-4π/3
前斜右:2   Θ-2π/3
後ろ:3   Θ
*/

int16_t pwm[3] = {0, 0, 0}; // それぞれの出力
double theta = 0; // ps4コントローラーの左スティックの角度
double L_x = 0; // ps4コントローラーの左スティックのx座標
double L_y = 0; // ps4コントローラーの左スティックのy座標
double R_x = 0; // ps4コントローラーの右スティックのx座標
double R_y = 0; // ps4コントローラーの右スティックのy座標

int speed = 10; // ここでスピードを調整する

constexpr uint32_t can_id = 30;
BufferedSerial pc{PB_6, PA_10, 4800};
CAN can{PA_11, PA_12, (int)1e6};
CANMessage msg;
Timer timer;

// シリアル通信で受け取ったデータの改行をなくすための関数
void readUntilPipe(char *output_buf, int output_buf_size) {
    char buf[20];
    int output_buf_index = 0;
    while (1) {
        if (pc.readable()) {
            ssize_t num = pc.read(buf, sizeof(buf) - 1); // -1 to leave space for null terminator
            buf[num] = '\0';
            for (int i = 0; i < num; i++) {
                if (buf[i] == '|') {
                    output_buf[output_buf_index] = '\0';
                    return;
                } else if (buf[i] != '\n' && output_buf_index < output_buf_size - 1) {
                    output_buf[output_buf_index++] = buf[i];
                }
            }
        }
        if (output_buf_index >= output_buf_size - 1) { // Prevent buffer overflow
            output_buf[output_buf_index] = '\0';
            return;
        }
    }
}
void canSend()
{
    while (1)
    {

        int16_t outputRightInt16 = static_cast<int16_t>(pwm[0]);
        DATA[0] = outputRightInt16 >> 8;   // MSB
        DATA[1] = outputRightInt16 & 0xFF; // LSB

        int16_t outputLeftInt16 = static_cast<int16_t>(pwm[1]);
        DATA[2] = outputLeftInt16 >> 8;   // MSB
        DATA[3] = outputLeftInt16 & 0xFF; // LSB

        int16_t outputLeftInt16 = static_cast<int16_t>(pwm[2]);
        DATA[2] = outputLeftInt16 >> 8;   // MSB
        DATA[3] = outputLeftInt16 & 0xFF; // LSB

        CANMessage msg0(0x200, DATA, 8);
        can.write(msg0);

    }
}

int main() {
    char output_buf[20]; // 出力用のバッファを作成します
    while (1) {
        readUntilPipe(output_buf, sizeof(output_buf)); // '|'が受け取られるまでデータを読み込みます
        
        if (strncmp(output_buf, "L3_x:", 5) == 0) {
            char *dataPointer = output_buf + 5; // "L3_x:"の後の文字列の先頭ポインタを取得
            L_x = atoi(dataPointer); // 数字部分を読み取る
        } else if (strncmp(output_buf, "L3_y:", 5) == 0) {
            char *dataPointer = output_buf + 5; // "L3_y:"の後の文字列の先頭ポインタを取得
            L_y = atoi(dataPointer); // 数字部分を読み取る
        } else if (output_buf[0] == 'p') { // pが送られてきた
            pwm[0] = pwm[1] = pwm[2] = 0; // 出力を停止する
        }
        
        theta = atan2(L_y, L_x) * RAD_TO_DEG; // radianからdegreeに変換したやつをthetaに代入
        
        pwm[0] = speed * cos(theta - 4 * M_PI / 3);
        pwm[1] = speed * cos(theta - 2 * M_PI / 3);
        pwm[2] = speed * cos(theta);
        
        // pwm[0], pwm[1], pwm[2]を送信する
        CANMessage msg0(can_id, (const uint8_t *)pwm, sizeof(pwm));
        can.write(msg0);
    }
}
