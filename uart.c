
/**
* @file uart.c
*/
#include <termios.h>
#include <stdio.h>
#include <pthread.h>
#include <stdint.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <strings.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>
#include <sys/select.h>
#include <string.h>
#include <signal.h>

int set_uart_attr(int fd, int nSpeed, int nBits, char nEvent, int nStop)
{
    struct termios newtio, oldtio;
    /*保存测试现有串口参数设置，在这里如果串口号等出错，会有相关的出错信息*/
    if (tcgetattr(fd, &oldtio) != 0)
    {
        perror("SetupSerial 1");
        return -1;
    }
    bzero(&newtio, sizeof(newtio));

    /*步骤一，设置字符大小*/
    newtio.c_cflag |= CLOCAL | CREAD;
    newtio.c_cflag &= ~CSIZE;

    /*设置停止位*/
    switch (nBits)
    {
        case 7:
            newtio.c_cflag |= CS7;
            break;
        case 8:
            newtio.c_cflag |= CS8;
            break;
    }

    /*设置奇偶校验位*/
    switch (nEvent)
    {
        case 'O':    //奇数
            newtio.c_cflag |= PARENB;
            newtio.c_cflag |= PARODD;
            newtio.c_iflag |= (INPCK | ISTRIP);
            break;
        case 'E':    //偶数
            newtio.c_iflag |= (INPCK | ISTRIP);
            newtio.c_cflag |= PARENB;
            newtio.c_cflag &= ~PARODD;
            break;
        case 'N':    //无奇偶校验位
            newtio.c_cflag &= ~PARENB;
            break;
    }

    /*设置波特率*/
    switch (nSpeed)
    {
        case 2400:
            cfsetispeed(&newtio, B2400);
            cfsetospeed(&newtio, B2400);
            break;
        case 4800:
            cfsetispeed(&newtio, B4800);
            cfsetospeed(&newtio, B4800);
            break;
        case 9600:
            cfsetispeed(&newtio, B9600);
            cfsetospeed(&newtio, B9600);
            break;
        case 115200:
            cfsetispeed(&newtio, B115200);
            cfsetospeed(&newtio, B115200);
            break;
        case 460800:
            cfsetispeed(&newtio, B460800);
            cfsetospeed(&newtio, B460800);
            break;
        default:
            cfsetispeed(&newtio, B9600);
            cfsetospeed(&newtio, B9600);
            break;
    }

    /*设置停止位*/
    if (nStop == 1)
    {
        newtio.c_cflag &= ~CSTOPB;
    }
    else if (nStop == 2)
    {
        newtio.c_cflag |= CSTOPB;
    }

    /*设置等待时间和最小接收字符*/
    newtio.c_cc[VTIME] = 0;
    newtio.c_cc[VMIN]  = 0;

    /*处理未接收字符*/
    tcflush(fd, TCIFLUSH);

    /*激活新配置*/
    if ((tcsetattr(fd, TCSANOW, &newtio)) != 0)
    {
        perror("com set error");
        return -1;
    }

    printf("set done!\n");
    return 0;
}

/*打开串口函数*/
int open_uart(const char* device_name)
{
    int fd;

    fd = open(device_name, O_RDWR | O_NOCTTY | O_NDELAY);
    if (-1 == fd)
    {
        perror("Can't Open Serial Port");
        return (-1);
    }

    /*恢复串口为阻塞状态*/
    if (fcntl(fd, F_SETFL, 0) < 0)
    {
        printf("fcntl failed!\n");
    }
    else
    {
        printf("fcntl=%d\n", fcntl(fd, F_SETFL, 0));
    }

    /*测试是否为终端设备*/
    if (isatty(STDIN_FILENO) == 0)
    {
        printf("standard input is not a terminal device\n");
    }
    else
    {
        printf("isatty success!\n");
    }

    printf("fd-open=%d\n", fd);
    return fd;
}

void* read_data(void* arg)
{
    char     buff[1024] = {0};
    uint32_t count      = 0;
    int      fd         = *((int*) arg);
    int      ret;

    fd_set fds;
    FD_ZERO(&fds);
    FD_SET(fd, &fds);
    while (1)
    {
        memset(buff,'\0',sizeof(buff));
        ret = select(fd + 1, &fds, NULL, NULL, NULL);
        if (ret <= 0)
        {
            continue;
        }
        ret = read(fd, buff, 1024);
        if (ret > 0)
        {
            count++;
            printf("%s", buff);
        }
        else
        {
            printf("read error\n");
        }
    }

    return arg;
}

void* write_data(void* arg)
{
    char buff[1024] = {0};
    int  fd         = *((int*) arg);
    int  ret;

    while (1)
    {
        printf("\nplease input:");
        fgets(buff, 1024, stdin);
        int ret = write(fd, buff, strlen(buff) + 1);
        if (ret > 0)
        {
            printf("send %d bytes\n", ret);
        }
        else
        {
            printf("write error\n");
        }
    }
}

int stop_flag = 0;

void signal_handle(int signum)
{
    if (signum == SIGINT)
    {
        printf("recv SIGINT\n");
        stop_flag = 1;
    }
}

int main()
{
    int       ret = 0;
    int       fd;
    char      buf[1024] = {0};
    pthread_t read_thread, write_thread;

    fd = open_uart("/dev/ttyUSB0");
    if (fd == -1)
    {
        printf("open failed\n");
        return -1;
    }

    ret = set_uart_attr(fd, 115200, 8, 'N', 1);
    if (ret != 0)
    {
        printf("set opt failed\n");
        return -1;
    }
    signal(SIGINT, signal_handle);

    pthread_create(&read_thread, NULL, read_data, (void*) &fd);
    pthread_create(&write_thread, NULL, write_data, (void*) &fd);

    while (stop_flag != 1)
    {
        sleep(1);
    }

    pthread_cancel(read_thread);
    pthread_cancel(write_thread);
    printf("pthread cancel\n");

    close(fd);

    return ret;
}
