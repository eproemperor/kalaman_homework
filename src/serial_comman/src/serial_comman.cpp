#include<serial_comman/serial_comman.h>

#include <fcntl.h>      // 文件控制定义
#include <unistd.h>     // UNIX标准函数定义
#include <termios.h>    // 终端I/O控制定义
#include <cstring>      // 字符串操作
#include <iostream>     // 错误输出

#define BAUDRATE B115200

SerialComm::SerialComm(const std::string& port) 
    : fd(-1)
    , port_name(port)
    , is_open(false)
    , last_error("") {
}

SerialComm::~SerialComm() {
    if (is_open) {
        close();
    }
}

bool SerialComm::open() {
    // 打开串口设备
    fd = ::open(port_name.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd == -1) {
        last_error = "无法打开串口设备: " + port_name;
        return false;
    }
    
    // 配置串口参数
    if (!configurePort()) {
        ::close(fd);
        fd = -1;
        return false;
    }
    
    is_open = true;
    last_error = "";
    return true;
}

void SerialComm::close() {
    if (fd != -1) {
        ::close(fd);
        fd = -1;
    }
    is_open = false;
}

bool SerialComm::configurePort() {
    struct termios options;
    
    // 获取当前串口配置
    if (tcgetattr(fd, &options) != 0) {
        last_error = "无法获取串口配置";
        return false;
    }
    
    cfsetispeed(&options, BAUDRATE);
    cfsetospeed(&options, BAUDRATE);
    
    // 控制模式标志
    options.c_cflag |= (CLOCAL | CREAD);    // 启用接收器，忽略调制解调器控制线
    options.c_cflag &= ~CSIZE;               // 清除数据位设置
    options.c_cflag |= CS8;                   // 8位数据
    options.c_cflag &= ~PARENB;               // 无校验位
    options.c_cflag &= ~CSTOPB;               // 1位停止位
    options.c_cflag &= ~CRTSCTS;              // 禁用硬件流控制
    
    // 本地模式标志
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);  // 原始输入模式
    
    // 输入模式标志
    options.c_iflag &= ~(IXON | IXOFF | IXANY);          // 禁用软件流控制
    options.c_iflag &= ~(INLCR | ICRNL | IGNCR);         // 禁用换行符转换
    
    // 输出模式标志
    options.c_oflag &= ~OPOST;                           
    
    // 设置读取超时
    options.c_cc[VMIN] = 0;     
    options.c_cc[VTIME] = 10;   
    
    tcflush(fd, TCIOFLUSH);
    
    if (tcsetattr(fd, TCSANOW, &options) != 0) {
        last_error = "无法设置串口配置";
        return false;
    }
    
    return true;
}

bool SerialComm::sendTurnCommand(float angle) {
    if (!is_open) {
        last_error = "串口未打开";
        return false;
    }
    
    // 1字节ID + 4字节float
    unsigned char buffer[5];
    buffer[0] = 0x01;  // 转向指令ID
    
    // 将float转换为字节数组
    unsigned char* float_bytes = reinterpret_cast<unsigned char*>(&angle);
    for (int i = 0; i < 4; i++) {
        buffer[1 + i] = float_bytes[i];
    }
    
    // 发送数据
    ssize_t bytes_written = write(fd, buffer, sizeof(buffer));
    if (bytes_written != sizeof(buffer)) {
        last_error = "发送转向指令失败";
        return false;
    }
    
    // 等待数据发送完成
    tcdrain(fd);
    
    last_error = "";
    return true;
}

bool SerialComm::sendFireCommand() {
    
    unsigned char buffer[1];
    buffer[0] = 0x02;  // 开火指令ID
    
    // 发送数据
    ssize_t bytes_written = write(fd, buffer, sizeof(buffer));
    if (bytes_written != sizeof(buffer)) {
        last_error = "发送开火指令失败";
        return false;
    }
    
    // 等待数据发送完成
    tcdrain(fd);
    
    last_error = "";
    return true;
}

SerialComman::SerialComman() : Node("serialcomman"){

    //command_pub_ = this->create_pubilsher<>()
    command_sub = this->create_subscription<vision_kalman_filter::msg::Serialcommand>(
        "control_command",
        10,
        std::bind(&SerialComman::SerialCallback,this,std::placeholders::_1)
    );
    

    this->declare_parameter<std::string>("serial_port", "/dev/pts/7");
    serial_comm_ = std::make_shared<SerialComm>(this->get_parameter("serial_port").as_string());

    RCLCPP_INFO(this->get_logger(),"串口发送接收节点已启动");


    Serial_ok_ = Serial_.open();
        if (!Serial_ok_) {
            RCLCPP_WARN(this->get_logger(), "串口打开失败，请检查权限");
        } else {
            RCLCPP_INFO(this->get_logger(), "串口打开成功");
        }

}

void SerialComman::SerialCallback(const vision_kalman_filter::msg::Serialcommand::SharedPtr msg){
        //RCLCPP_INFO(
        //    this->get_logger(),
        //    "收到开火信息，方向：%f,开火:%d",
        //    msg->angle,
        //    msg->isshout
        //);

        std::lock_guard<std::mutex> lock(mutex_);

        angle = msg->angle;
        isshout = msg->isshout;

        Serial_.sendTurnCommand(angle);
        usleep(10000);
        Serial_.sendFireCommand();
}

SerialComman::~SerialComman(){

}


int main(int argc,char* argv[]){

    rclcpp::init(argc, argv);
    auto node = std::make_shared<SerialComman>();
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;

}