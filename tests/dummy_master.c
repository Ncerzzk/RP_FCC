
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <asm/termbits.h>
#include <assert.h>

#include "../minihdlc/minihdlc.h"

// compile: gcc dummy_master.c ../minihdlc/minihdlc.c -o dummy_master
// run: ./dummy_master /dev/ttyUSB0

int uart_fd_init(char *device, int baudrate)
{
    int fd;
    struct termios2 tty;

    fd = open(device, O_RDWR | O_NOCTTY | O_NONBLOCK);
    assert(fd >= 0);

    assert(ioctl(fd, TCGETS2, &tty) == 0);

    tty.c_cflag &= ~CBAUD;
    tty.c_cflag |= BOTHER;
    tty.c_ospeed = baudrate;
    tty.c_ispeed = baudrate;

    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;     // 8 数据位
    tty.c_cflag &= ~PARENB; // 无校验
    tty.c_cflag &= ~CSTOPB; // 1 停止位

    tty.c_cflag |= (CLOCAL | CREAD);                // 忽略调制解调器控制 | 启用接收器
    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // 原始输入模式
    tty.c_oflag &= ~OPOST;                          // 原始输出模式

    // 清空缓冲区
    ioctl(fd, TCFLSH, TCIOFLUSH);

    assert(ioctl(fd, TCSETSF2, &tty) == 0);

    return fd;
}

FILE *uart_file_init(int fd)
{
    FILE *serial_stream = fdopen(fd, "w+");
    assert(serial_stream);
    setvbuf(serial_stream, NULL, _IONBF, 0);

    return serial_stream;
}

FILE *stream;
void send_char(uint8_t data)
{
    fputc(data, stream);
}

int main(int argc, char *argv[])
{
    int fd = uart_fd_init(argv[1], 115200);

    stream = uart_file_init(fd);

    minihdlc_init(send_char, NULL);

    printf("prepare to send!\n");
    minihdlc_send_frame("hello", sizeof("hello"));

    printf("hello,world!\n");

    fclose(stream);

    printf("finished.\n");
    return 0;
}