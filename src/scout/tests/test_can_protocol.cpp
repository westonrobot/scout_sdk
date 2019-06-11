#include "scout/scout_can_protocol.hpp"

using namespace wescore;

int main()
{
    // uint8_t data1[8] = {1, 2, 3};
    // uint8_t data2[8] = {0, 1, 2, 3, 4, 5, 6, 7};

    // std::cout << "control modes: " << SMotionControlDef::REMOTE_MODE
    //           << " max speed " << static_cast<int>(SMotionControlDef::max_angular_speed) << std::endl;

    // std::cout << "motion ctrl msg count: " << static_cast<int>(MotionControlMessage::count) << std::endl;
    // std::cout << "motion ctrl msg count: " << static_cast<int>(MotionControlMessage::count) << std::endl;

    // std::cout << "light ctrl msg count: " << static_cast<int>(SLightControlMessage::count) << std::endl;
    // std::cout << "light ctrl msg count: " << static_cast<int>(SLightControlMessage::count) << std::endl;

    // MotionControlMessage msg1(data1);
    // SLightControlMessage msg2(data2);

    // MotionControlMessage msg3(data1);
    // SLightControlMessage msg4(data2);

    // SLightControlMessage msg5 = msg4;
    // SLightControlMessage msg6 = SLightControlMessage(data2);

    // std::cout << "motion ctrl msg count: " << static_cast<int>(MotionControlMessage::count) << std::endl;
    // std::cout << "light ctrl msg count: " << static_cast<int>(SLightControlMessage::count) << std::endl;

    // MotionControlMessage msg7(0, 0);
    // MotionControlMessage msg8(0.15, 0);
    // MotionControlMessage msg9(0, 0.07853);

    // std::cout << msg7 << std::endl;
    // std::cout << msg8 << std::endl;
    // std::cout << msg9 << std::endl;

    // for (int i = 0; i < 260; ++i)
    // {
        MotionControlMessage msg;
        msg.linear_velocity = 0.15;
        msg.angular_velocity = 0.0;
        msg.fault_clear_flag = FaultClearFlag::NO_FAULT;
        msg.gen();
        // std::cout << msg << std::endl;

        std::cout << "ID: " << std::hex << msg.id << ", DLC: " << static_cast<int>(msg.dlc) << ", Data: ";
        for (int i = 0; i < 8; ++i)
            std::cout << static_cast<int>(msg.data[i]) << " ";
        std::cout << ", Checksum: " << std::dec << static_cast<int>(msg.GenCheckSum()) << std::endl;
        
    // }

    // send CAN frame
    // struct can_frame frame;
    // frame.can_id = 0x123;
    // frame.can_dlc = 2;
    // frame.data[0] = 0x11;
    // frame.data[1] = 0x23;
    // canbus->send_frame(frame);

    // receive CAN frame
    // void default_receive_callback(can_frame * rx_frame)
    // {
    // process rx_frame
    // }

    return 0;
}