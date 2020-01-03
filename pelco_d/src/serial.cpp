#include "serial.h"

Serial::Serial(std::string serialport, int baud)
{
    // Standard struct for holding port data/settings
    struct termios toptions;

    // Read/write, no delays
    int fd = open(serialport.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd == -1)
    {
        perror("Serial: Unable to open port ");
        // return -1;
    }

    if (tcgetattr(fd, &toptions) < 0)
    {
        perror("Serial: Couldn't get term attributes");
        // return -1;
    }

    speed_t brate = baud;

    // Simple switch for the baudrate selection
    switch (baud)
    {
    case 4800:
        brate = B4800;
        break;
    case 9600:
        brate = B9600;
        break;
    case 19200:
        brate = B19200;
        break;
    case 38400:
        brate = B38400;
        break;
    case 57600:
        brate = B57600;
        break;
    case 115200:
        brate = B115200;
        break;
    }

    cfsetispeed(&toptions, brate);
    cfsetospeed(&toptions, brate);

    // 8N1, no hw flow control, configured for read access as well as write
    toptions.c_cflag &= ~(PARENB | CSTOPB | CSIZE | CRTSCTS | CREAD | CLOCAL);
    toptions.c_cflag |= CS8;

    // turn on s/w flow ctrl
    toptions.c_iflag |= (IXON | IXOFF | IXANY);
    // make sure the CR and NL aren't mapped to each other on input
    toptions.c_iflag &= ~(INLCR | ICRNL);
    // THE ABOVE OPTION'S EXISTENCE IS NECESSARY. IF YOU REMOVE IT
    //  YOU WILL SPEND HOURS PULLING YOUR HAIR OUR TROUBLESHOOTING
    //  STRINGS THAT JUST WON'T PARSE BECAUSE '\r' IS REPLACED WITH '\n'
    //  by the cheap serial->USB adapter you have or the different
    //  default port settings of your kernel. This bit me.

    // make raw output as well
    toptions.c_oflag &= ~OPOST;
    // make sure the CR and NL aren't mapped to each other on output
    toptions.c_oflag &= ~(INLCR | ICRNL);
    // make raw input, not line-based canonical
    toptions.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    // Not necessary, but peace of mind to set timeouts
    toptions.c_cc[VMIN] = 0;
    toptions.c_cc[VTIME] = 2;

    if (tcsetattr(fd, TCSAFLUSH, &toptions) < 0)
    {
        // Self explanatory, attempts to set the port attribs
        perror("Serial: Couldn't set term attributes");
        // return -1;
    }

    this->file_descriptor = fd;

    // wait until we're done initializing o do anything else.
    sleep(1);
}

Serial::~Serial()
{
    close(this->file_descriptor);
}

// This shunts any piled-up/remaining data in the serial
// device's buffers into oblivion, guaranteeing clean input.
void Serial::clear()
{
    ssize_t n = 1;
    char nothing = '\0';

    while (n > 0)
        n = read(this->file_descriptor, &nothing, 1);
    // If a byte is read n is >0, so
    // it continues until flushed
}

// void Serial::clearBuffer()
// {
//     //clears any hanging values from the Odroid's serial comms buffer
//     char buffer[512] = "";
//     int bytes_read = read(this->file_descriptor, &buffer, sizeof(buffer));
//     if (bytes_read > 0)
//     {
//         buffer[bytes_read] = '\0';
//         printf("buffer (%d bytes): %s\n", bytes_read, buffer);
//         fflush(stdout);
//     }
// }

ssize_t Serial::Write(const char *msg, size_t bytes)
{
    return write(this->file_descriptor, msg, bytes);
}

// attempt to read up to `bytes` number of bytes
ssize_t Serial::Read(char *buffer, size_t bytes)
{
    // let's hope the user passes in an appropriate number of bytes
    bzero(buffer, bytes);
    size_t bytes_read = read(this->file_descriptor, buffer, bytes);
    buffer[bytes_read] = '\0';
    return bytes_read;
}

// Keep reading until we read `bytes` number of bytes
size_t Serial::ReadBlocking(char *buffer, size_t bytes)
{
    // let's hope the user passes in an appropriate number of bytes
    bzero(buffer, bytes);
    size_t bytes_read = 0;
    // it's important that tmp be `ssize_t` rather than `size_t`
    // as `ssize_t` can be *signed* -- read returns -1 if it didn't read
    // anything...
    ssize_t tmp;

    // try to read bytes number of bytes
    while (bytes_read < bytes)
    {
        // offset the buffer the appropriate amount so we don't overwrite
        // something we'v already written
        tmp = read(this->file_descriptor, buffer + bytes_read, bytes - bytes_read);

        if (tmp > 0)
        {
            bytes_read += tmp;
        }
    }

    return bytes_read;
}