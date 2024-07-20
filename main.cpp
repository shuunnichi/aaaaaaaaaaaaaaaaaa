#include "mbed.h"
#include "PID.hpp"
// #include "firstpenguin.hpp"
constexpr uint32_t can_id = 35;
BufferedSerial pc{USBTX, USBRX, 250000};

float kp = 0.8f, ki = 3.0f, kd = 0.00f;
float sampleTime = 0.1f; // サンプル時間 (秒)

CAN can{PA_11, PA_12, (int)1e6};
// CAN can{PB_12, PB_13, (int)1e6};
CANMessage msg;

Timer timer;

double targetXDistance = 500.0; // 目標x方向の距離
double targetYDistance = 500.0; // 目標y方向の距離

int jidou = 0;
PID pidX(kp, ki, kd, sampleTime), pidY(kp, ki, kd, sampleTime);

struct FirstPenguin
{
    static constexpr int max = INT16_MAX;
    uint32_t send_id;
    int16_t pwm[4] = {};
    struct
    {
        int32_t enc;
        void set(const uint8_t data[8])
        {
            memcpy(&enc, data, sizeof(enc));
        }
    } receive[4] = {};
    void read(const CANMessage &msg)
    {
        if (msg.format == CANStandard && msg.type == CANData && msg.len == 8 && send_id < msg.id && msg.id <= send_id + 5)
        {
            // printf("Received CAN message with ID: %d\n", msg.id); // デバッグメッセージ
            receive[msg.id - send_id - 1].set(msg.data);
        }
    }
    bool send()
    {
        bool result = can.write(CANMessage{send_id, reinterpret_cast<const uint8_t *>(pwm), 8});
        if (result)
        {
            // printf("CAN message sent successfully\n"); // デバッグメッセージ
        }       
        else
        {
            // printf("Failed to send CAN message\n"); // デバッグメッセージ
        }
        return result;
    }
};

FirstPenguin penguin{can_id};

int main()
{
    printf("\nsetup\n");
    timer.start();
    auto pre = timer.elapsed_time();
    while (1)
    {

        // void read(const CANMessage &msg);
        // {
        //     if (msg.format == CANStandard && msg.type == CANData && msg.len == sizeof(receive[0]) && send_id < msg.id &&
        //         msg.id <= send_id + 5)
        //     {
        //         receive[msg.id - send_id - 1].set(msg.data);
        //     }
        // }

        char buf;
        if (pc.readable() && pc.read(&buf, sizeof(buf)))
        {
            if (buf == 's' && jidou == 0) //  /henkouten1
            {
                jidou = 1;                       // autoを1に設定します
                printf("program start\n");       // プログラム開始のメッセージを出力します
                double xDistanceTravelled = 0.0; // x方向に移動した距離を初期化します
                double yDistanceTravelled = 0.0; // y方向に移動した距離を初期化します
                timer.start();                   // タイマーを開始
                auto pre = timer.elapsed_time();
                while (xDistanceTravelled < targetXDistance && yDistanceTravelled < targetYDistance)
                {
                    auto now = timer.elapsed_time();

                    if (can.read(msg))
                    {
                        penguin.read(msg);
                    }

                    if (now - pre > 20ms)
                    {
                        for (auto e : penguin.receive)
                            printf("% 5ld\t", e.enc);

                        printf("\n");
                        for (auto &e : penguin.pwm)
                            e = penguin.max / 4;
                        penguin.send();
                        pre = now;
                    }
                    xDistanceTravelled = (penguin.receive[0].enc+penguin.receive[1].enc-penguin.receive[2].enc-penguin.receive[3].enc)/4;
                    yDistanceTravelled = (penguin.receive[0].enc-penguin.receive[1].enc-penguin.receive[2].enc+penguin.receive[3].enc)/4;
                    // xDistanceTravelled = e.enc;
                    // yDistanceTravelled = e.enc;
                    // xDistanceTravelled = encoder1;
                    // yDistanceTravelled = encoder1;
                    float output_x = pidX.calculate(targetXDistance, xDistanceTravelled);
                    float output_y = pidY.calculate(targetYDistance, yDistanceTravelled);
                    // 前進値と左右の移動値を指定してロボットを制御します
                    // メカナムホイールの特性を利用して、各ホイールに対して独立した速度を設定します

                    const float n = 10000.0;                                                                                                  // この値は例です。実際の値に応じて調整してください。
                    // printf("x: %d, y: %d,nowx:%d,nowy:%d\n", int(output_x), int(output_y), int(xDistanceTravelled), int(yDistanceTravelled)); // xとyの値を出力します
                    if (abs(output_x + output_y) > n)
                    {
                        if (output_x + output_y > 0)
                        {
                            penguin.pwm[0] = n;
                        }
                        else
                        {
                            penguin.pwm[0] = -n;
                        }
                    }
                    else
                    {
                        penguin.pwm[0] = abs(output_x + output_y);
                    }
                    if (abs(output_x - output_y) > n)
                    {
                        if (output_x - output_y > 0)
                        {
                            penguin.pwm[1] = n;
                        }
                        else
                        {
                            penguin.pwm[1] = -n;
                        }
                    }
                    else
                    {
                        penguin.pwm[1] = abs(output_x - output_y);
                    }
                    if (abs(-output_x - output_y) > n)
                    {
                        if (-output_x - output_y > 0)
                        {
                            penguin.pwm[2] = n;
                        }
                        else
                        {
                            penguin.pwm[2] = -n;
                        }
                    }
                    else
                    {
                        penguin.pwm[2] = abs(-output_x - output_y);
                    }
                    if (abs(output_y - output_x) > n)
                    {
                        if (output_y - output_x > 0)
                        {
                            penguin.pwm[3] = n;
                        }
                        else
                        {
                            penguin.pwm[3] = -n;
                        }
                    }
                    else
                    {
                        penguin.pwm[3] = abs(output_y - output_x);
                    }

                    penguin.send();
                }
                penguin.pwm[0] = 0;
                penguin.pwm[1] = 0;
                penguin.pwm[2] = 0;
                penguin.pwm[3] = 0;


                penguin.send();
            }
        }
    }
}