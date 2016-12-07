#include <stdio.h>
#include <errno.h>
#include <fcntl.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <pthread.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <sys/types.h>
#include <netdb.h>

int fd;
int socket_fd;
int socketConnection;

pthread_t readThread;

int set_interface_attribs (int fd, int speed, int parity)
{
    struct termios tty;
    memset (&tty, 0, sizeof tty);
    if (tcgetattr (fd, &tty) != 0) {
        printf ("error %d from tcgetattr", errno);
        return -1;
    }
    cfsetospeed (&tty, speed);
    cfsetispeed (&tty, speed);
    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_iflag &= ~IGNBRK;
    tty.c_lflag = 0;
    tty.c_oflag = 0;
    tty.c_cc[VMIN]  = 0;
    tty.c_cc[VTIME] = 5;
    tty.c_iflag &= ~(IXOFF | IXON | IXANY);
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~(PARENB | PARODD);
    tty.c_cflag |= parity;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;
    if (tcsetattr (fd, TCSANOW, &tty) != 0) {
        printf ("error %d from tcsetattr", errno);
        return -1;
    }
    return 0;
}

void set_blocking (int fd, int should_block)
{
    struct termios tty;
    memset (&tty, 0, sizeof tty);
    if (tcgetattr (fd, &tty) != 0) {
        printf ("error %d from tggetattr", errno);
        return;
    }
    tty.c_cc[VMIN]  = should_block ? 1 : 0;
    tty.c_cc[VTIME] = 5;
    if (tcsetattr (fd, TCSANOW, &tty) != 0) {
        printf ("error %d setting term attributes", errno);
    }
}

void printHexString(unsigned char * input, int length)
{
    int i;
    for (i = 0; i < length; i++) {
        printf("%02X", (unsigned char) input[i]);
    }
    printf("\n");
}

void * fromSerial()
{
    char buffer[1024];
    int received = 0;
    while(1) {
        memset(buffer, 0x00, 1024);
        received = read (fd, buffer, 1024);
        if (received > 0) {
            write(socketConnection, buffer, received);
            printf("Readserial: ");
            printHexString(buffer, received);
        }
    }
    return NULL;
}

int main(int argc, char ** argv)
{
    struct sockaddr_in socketInfo;
    int portNumber = 4815;
    int iret1;
    int rc;
    int connected = 0;
    char buffer[1024];

    // init serial port
    char *portname = "/dev/ttyAMA0";
    fd = open (portname, O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0) {
        printf ("error %d opening %s: %s\n", errno, portname, strerror (errno));
        return 1;
    }

    set_interface_attribs (fd, B9600, 0);
    set_blocking (fd, 0);

    if((socket_fd = socket(AF_INET, SOCK_STREAM, IPPROTO_IP)) < 0) {
        close(socket_fd);
        return 1;
    }

    bzero(&socketInfo, sizeof(struct sockaddr_in));
    socketInfo.sin_family = AF_INET;
    socketInfo.sin_addr.s_addr = htonl(INADDR_ANY);
    socketInfo.sin_port = htons(portNumber);

    if( bind(socket_fd, (struct sockaddr *) &socketInfo, sizeof(struct sockaddr_in)) < 0) {
        close(socket_fd);
        perror("bind");
        return 1;
    }

    while(1) {
        listen(socket_fd, 1);
	if( (socketConnection = accept(socket_fd, NULL, NULL)) < 0) {
            close(socket_fd);
            return 1;
        }
        connected = 1;
        iret1 = pthread_create( &readThread, NULL, fromSerial, NULL);
        while(connected) {
            memset(buffer, 0x00, 1024);
            rc = recv(socketConnection, buffer, 1024, 0);
            if ( rc == 0 ) {
                printf("socket closed\n");
                connected = 0;
            } else if (rc == -1) {
                printf("socket error\n");
                close(socketConnection);
                connected = 0;
            }
            printf("Writeserial: ");
            printHexString(buffer, rc);
            write(fd, buffer, rc);
        }
        pthread_cancel(readThread);
    }
    close(fd);
    return 0;
}
