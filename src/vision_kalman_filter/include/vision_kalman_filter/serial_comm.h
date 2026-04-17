/**
 * 角度定义：
 * - 0°：水平向右（右侧水平线）
 * - 90°：垂直向上
 * - 180°：水平向左
 * - 角度范围：0° ~ 180°
 * - 负角度表示向下：例如 -90° 表示垂直向下
 */

#ifndef SERIAL_COMM_H
#define SERIAL_COMM_H

#include <string>

class SerialComm {
private:
    int fd;                 // 串口文件描述符
    std::string port_name;  // 串口设备路径
    bool is_open;          
    
public:
    SerialComm(const std::string& port = "/dev/pts/7");
    
    ~SerialComm();
    
    bool open();
    
    void close();
    
    bool isOpen() const { return is_open; }
    
    /**
     *  angle 角度值（度）
     *        - 0°：水平向右
     *        - 90°：垂直向上
     *        - 180°：水平向左
     *        - 负值：向下
     */
    bool sendTurnCommand(float angle);
    
    bool sendFireCommand();
    
    std::string getLastError() const { return last_error; }
    
private:
    std::string last_error;
    
    bool configurePort();
};

#endif // SERIAL_COMM_H
