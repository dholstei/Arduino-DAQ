//    serial driver for LabView
//    written by Danny Holstein

#ifdef WIN
 #define EXPORT __declspec(dllexport)
 #include <windows.h>
 #include <winuser.h>
 typedef long ulong;
#else
 #define EXPORT
 #define _GNU_SOURCE
 #define USB_POLL
#endif

#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#ifndef WIN
 #include <termios.h>
 #include <sys/ioctl.h>
 #include <poll.h>
 #include <sys/socket.h>
 #include <netdb.h>
#else
 DWORD dwRead;
 BOOL fWaitingOnRead = FALSE;
 OVERLAPPED osReader = {0};
// #include <winsock2.h>
#endif

#include <fcntl.h>
#include <errno.h>

#include <sys/types.h>

int open_socket(char *IP_address, int port);
int open_device( const char *dev);
int read_from_device(unsigned fd, char *buf, unsigned long buf_size, char TCP);
int write_to_device(unsigned fd, char *buf, unsigned long buf_size, char TCP);
int clear_buffers (int fd);
int set_interface_attribs(int fd, unsigned long baud, 
        unsigned short DataBits, unsigned short StopBits, unsigned short Parity, 
        unsigned char TerminationChar);

fd_set rfds; struct timeval tv, tv_zero = {0,0}; int retval;

#define PORT_CHECK(port_hdl) \
    {if (port_hdl == -1)\
        {sprintf(COMM_error, "%s\n", strerror( errno )); return(-1);}}
#define TCP_CHECK(port, error) {if (port == -1)\
 {sprintf(COMM_error, "%s\n", error); return(-1);}}
#define TCP_CHECK_(port, error) {if (port == -1)\
 {sprintf(COMM_error, "%s\n", error); return;}}
#define RTN_ERROR {sprintf(COMM_error, "%s\n", strerror( errno )); return(-1);}
#define RTN_ERROR_V {sprintf(COMM_error, "%s\n", strerror( errno )); return;}
#define TIMO_ERROR(a) \
    {sprintf(COMM_error, "Timeout error on Serial\nPort: %2d\n", (a));\
        return(-1);}
#define TIMO_ERROR_V(a) \
    {sprintf(COMM_error, "Timeout error on Serial\nPort: %2d\n", (a));\
		return;}
#define PROWRITE(ud, cmd)     \
    (void) write_to_device(fd, cmd, TCP )
/* 
 * ---macros to handle differences between Win and Posix serial communications
 */
 char dgh_text[1024];
 #define SER_PORT_BUF_SZ 0x20000
 
#ifndef WIN	// ---Linux/Posix stuff
 #define PORT_SER_OPEN(dev)		open(dev,O_RDWR|O_NOCTTY|O_SYNC);
 #define USLEEP(usec)			usleep(usec)
 #define DGH_DEBUG(buf) printf("func = %s\nline = %d\nbuffer = %s\n", __func__, __LINE__, (char *) buf);
 #define PROCLOSE(fd)     \
		{close(fd);fd = 0;}
#else	// ---Win stuff
 #define _TCP 0
 #define PORT_SER_OPEN(dev)		(unsigned) CreateFile(dev, GENERIC_READ | GENERIC_WRITE, 0, 0, OPEN_EXISTING, 0, 0);\
									(void) SetupComm((void *) port, SER_PORT_BUF_SZ, 0x2000);
 #define USLEEP(usec)			Sleep(usec/1000)
 #define DGH_DEBUG(buf) sprintf(dgh_text, "func = %s\nline = %d\nbuffer = %s\n", __func__, __LINE__, (CHAR *) buf );\
						if (MessageBox(0, dgh_text, "danny debug", MB_OKCANCEL) == IDCANCEL) return(-1);
 #define PROCLOSE(fd)     \
		{if(_TCP) {close(fd);fd = 0;}\
		else {CloseHandle((void *) fd);fd = 0;}}
#endif

int timo[]     = {500, 1000, 2000, 3000},
    timo_ser[] = {550, 1200, 2300, 3300};
#define S_TV(a) timo_ser[a]/1000
#define MS_TV(a) timo_ser[a] % 1000

#define MAX(a,b) (( a > b ) ? a : b )
#define MIN(a,b) (( a < b ) ? a : b )

volatile int last_pad = 0;
volatile char cnt_read = 0;

static int usleep_const = 20000;  /* 20000 for USB0, 100000 for USB1  */
#define IO_DLY 100000

char COMM_error[1024], cmd[1024];
FILE *dev_file;


char DGH_t[1024];

#define THIS_RTN_ERROR {sprintf(COMM_error, "Failure in serial read\n"); return(-1);}

int read_from_device(unsigned fd, char *buf, unsigned long buf_size, char TCP)
{  /*  Read data from a device into user buffer.  */
	int k=0, count;
	char *line_buf;
	line_buf = (char*) buf;
	
	count = buf_size;
	if (TCP) {
        if ((k = recv(fd, line_buf, count - 1, 0)) <= 0) TCP_CHECK(-1, "Data receive error");}
    else {
    /* Watch port to see when it has input. */
 #ifndef WIN
    struct pollfd fds; int i;
    fds.fd = fd; fds.events = POLLIN;
    switch (poll(&fds, 1, timo[2])) {
        case -1 :	// error
            THIS_RTN_ERROR
            break;
        case  0 :	//  timeout
            TIMO_ERROR(fd)
            break;
        default:	//  OK to read
            fcntl(fd, F_SETFL, FNDELAY);
            if (TCP) {
                while ((i=recv(fd, line_buf+k, count-k, 0))>0) {
                    k += i; USLEEP(100000);}}
            else {
                while ((i=read(fd, line_buf+k, count-k))>0) {k += i; USLEEP(usleep_const);}}
            fcntl(fd, F_SETFL, 0);
            line_buf[k] = 0;
            break;
    }
 #else
    DWORD dwEventMask, *dwIncommingReadSize;
    dwIncommingReadSize = malloc(sizeof(DWORD));
    int dwSize=0;
    char szBuf[10];
    if(!SetCommMask((void *) fd, EV_RXCHAR)) {THIS_RTN_ERROR}
    if (count>2048) {ReadFile((void *) fd, buf, count, dwIncommingReadSize, NULL); k=*dwIncommingReadSize;}
    else {
        do {
                if(ReadFile((void *) fd, szBuf, 1, dwIncommingReadSize, NULL) != 0) {
                    if(*dwIncommingReadSize > 0) {
                        for (k=0; k<*dwIncommingReadSize; k++) {*(line_buf+dwSize+k) = szBuf[k];}
                        dwSize += *dwIncommingReadSize;
                        }}
                else {THIS_RTN_ERROR;}
            } while(*dwIncommingReadSize > 0);
            k = dwSize;}
 #endif
    }
	return(k);
}

#undef THIS_RTN_ERROR
#define THIS_RTN_ERROR {sprintf(COMM_error, "Failure in serial write\n"); return(-1);}
int write_to_device( unsigned fd, char *buf, unsigned long buf_size, char TCP )
{  /*  Write data to a device from a user buffer.  */

    if (TCP) {
        /* Send the string to the TCP/GPIB "server" */
        if (send(fd, buf, buf_size, 0) != buf_size) TCP_CHECK(-1, "Data transmission error (TCP/GPIB)");
    }
    else {

#ifndef WIN
        if (write(fd, buf, buf_size) == -1) THIS_RTN_ERROR
#else
        unsigned long dwNumberOfBytesSent = 0, dwNumberOfBytesWritten, dwSize;
        dwSize = buf_size;

        while(dwNumberOfBytesSent < dwSize /*size of the buffer pszBuf*/)
        {
            if(WriteFile((void *) fd, &buf[dwNumberOfBytesSent], 1, &dwNumberOfBytesWritten, NULL) != 0)
            {
                if(dwNumberOfBytesWritten > 0) ++dwNumberOfBytesSent;
                else {THIS_RTN_ERROR;}
            }
            else {THIS_RTN_ERROR;}
        }
#endif
    }
    return(buf_size);
}

#undef THIS_RTN_ERROR
#define THIS_RTN_ERROR {sprintf(COMM_error, "Failure in serial set interface attribs\n"); return(-1);}
int set_interface_attribs(int fd, unsigned long baud, 
        unsigned short DataBits, unsigned short StopBits, unsigned short Parity, 
        unsigned char TerminationChar)
{
#ifndef WIN
    struct termios tty;
    memset (&tty, 0, sizeof tty);
    if (tcgetattr (fd, &tty) != 0)
    {
            printf ("error %d from tcgetattr", errno);
            return -1;
    }

    cfsetospeed (&tty, baud);
    cfsetispeed (&tty, baud);
    
    switch(DataBits) {
        case CS5  :
        case CS6  :
        case CS7  :
        case CS8  :
            break;
        default :
            DataBits = CS8;
            break;
    }

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | DataBits;
    // disable IGNBRK for mismatched speed tests; otherwise receive break
    // as \000 chars
    tty.c_iflag &= ~IGNBRK;         // disable break processing
    tty.c_lflag = 0;                // no signaling chars, no echo,
                                    // no canonical processing
    tty.c_oflag = 0;                // no remapping, no delays
    tty.c_cc[VMIN]  = 0;            // read doesn't block
    tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

    tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
                                    // enable reading
    
    tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
    switch(Parity) {
        case 1  :   // Odd
            tty.c_cflag |= PARODD;
        case 2  :   // Even
            tty.c_cflag |= PARENB;
            break;
        case 3  :   // Mark
            tty.c_cflag |= PARENB | CMSPAR | PARODD;
            break;
        case 4  :   // Space
            tty.c_cflag |= PARENB | CMSPAR;
            tty.c_cflag &= ~PARODD;
            break;
        case 0  :   // None
        default :
            break;
    }
    tty.c_cflag &= ~CSTOPB;
    switch(StopBits) {
        case 10  :   // 1.0
            break;
        case 15  :   // 1.5
        case 20  :   // 2.0
            tty.c_cflag |= CSTOPB;
            break;
        default :
            break;
    }
    tty.c_cflag &= ~CRTSCTS;

    if (tcsetattr (fd, TCSANOW, &tty) != 0)
        {printf ("error %d from tcsetattr", errno); return -1;}
#else
	DCB dcbConfig;

	if(GetCommState((void *) fd, &dcbConfig))
	{
	    // Change port parameters as needed
        dcbConfig.BaudRate        = baud;
        dcbConfig.ByteSize        = DataBits;
        dcbConfig.Parity          = Parity;
        switch(StopBits) {
            case 15  :   // 1.5
                dcbConfig.StopBits = ONE5STOPBITS;
                break;
            case 20  :   // 2.0
                dcbConfig.StopBits = TWOSTOPBITS;
                break;
            default :   // 1.0
                dcbConfig.StopBits = ONESTOPBIT;
                break;
        }
        dcbConfig.fParity         = FALSE;
        dcbConfig.fBinary         = TRUE;                     // Windows requires this to be TRUE

        // Enable RTS/CTS handshaking
        dcbConfig.fOutxCtsFlow    = TRUE;                     // CTS used for output flow control
        dcbConfig.fRtsControl     = RTS_CONTROL_HANDSHAKE;    // Enable RTS handshaking

        // Disable DTR/DSR handshaking
        dcbConfig.fOutxDsrFlow    = FALSE;                    // DSR not used for output flow control
        dcbConfig.fDsrSensitivity = FALSE;                    // Ignore DSR
        dcbConfig.fDtrControl     = DTR_CONTROL_ENABLE;       // Turn on DTR upon device open

        // Disable XON/XOFF handshaking
        dcbConfig.fOutX           = FALSE;                    // No outbound XON/XOFF flow control
        dcbConfig.fInX            = FALSE;                    // No inbound XON/XOFF flow control

        // Error handling
        dcbConfig.fErrorChar      = FALSE;                    // No replacement of bytes with parity error
        dcbConfig.fNull           = FALSE;                    // Don't discard NULL bytes
        dcbConfig.fAbortOnError   = FALSE;
	}

	else
		//Handle Error Condition

	if(!SetCommState((void *) fd, &dcbConfig))
		{printf ("error in SetCommState"); return -1;}
	        
		COMMTIMEOUTS cto;

        // Get port timeout parameters
        if ( GetCommTimeouts((void *) fd, &cto ) == 0 ) {THIS_RTN_ERROR}

        // A value of MAXDWORD, combined with zero values for both the
        // ReadTotalTimeoutConstant and ReadTotalTimeoutMultiplier members,
        // specifies that the read operation is to return immediately with the
        // bytes that have already been received, even if no bytes have been
        // received.

        cto.ReadIntervalTimeout           = MAXDWORD;
        cto.ReadTotalTimeoutMultiplier    = 0;
        cto.WriteTotalTimeoutMultiplier   = 0;
        cto.WriteTotalTimeoutConstant     = 0;

        // Set port timeout parameters
        if ( SetCommTimeouts((void *) fd, &cto ) == 0 ) {THIS_RTN_ERROR}
#endif
        return 0;
}

int clear_buffers (int fd)
{
#ifndef WIN
    tcflush(fd,TCIOFLUSH);
#else
	(void) PurgeComm((void *) fd, PURGE_RXCLEAR);
#endif
    return 0;
}

int open_device(const char *dev)
{
    int i, j, k=0, port=0;
    char servIP[128], TCP=0;
    if (sscanf(dev, "%[^:]:%d", servIP, &port) == 2) {
        port = open_socket(servIP, port); TCP=1;
        TCP_CHECK(port, "Connect failure");}
    else {
        port = PORT_SER_OPEN(dev);
#ifndef WIN
        if (port !=0 ) (void) set_interface_attribs (port, B9600, CS8, 0, 0, 0);
#else
        if (port !=0 ) (void) set_interface_attribs (port, CBR_9600, DATABITS_8, 0, 0, 0);
#endif
        PORT_CHECK(port);}

    return(port);
}

int close_device (int fd)
{
    PROCLOSE(fd);
    return 0;
}

int open_socket(char *IP_address, int port)
{
    int sock;                        /* Socket descriptor */
    struct sockaddr_in echoServAddr; /* Echo server address */
    unsigned short echoServPort;     /* Echo server port */
    char *servIP;                    /* Server IP address (dotted quad) */
    char *echoString;                /* String to send to echo server */
//     char echoBuffer[RCVBUFSIZE];     /* Buffer for echo string */
    unsigned int echoStringLen;      /* Length of string to echo */
    int bytesRcvd, totalBytesRcvd;   /* Bytes read in single recv() 
                                        and total bytes read */

    if (IP_address == NULL) return(-1);

    servIP = IP_address;             /* First arg: server IP address (dotted quad) */
//     echoString = argv[2];         /* Second arg: string to echo */

    echoServPort = port; /* Use given port */

    /* Create a reliable, stream socket using TCP */
    if ((sock = socket(PF_INET, SOCK_STREAM, IPPROTO_TCP)) < 0)
        return -1;

    /* Construct the server address structure */
    memset(&echoServAddr, 0, sizeof(echoServAddr));     /* Zero out structure */
    echoServAddr.sin_family      = AF_INET;             /* Internet address family */
    echoServAddr.sin_addr.s_addr = inet_addr(servIP);   /* Server IP address */
    echoServAddr.sin_port        = htons(echoServPort); /* Server port */

    /* Establish the connection to the echo server */
    if (connect(sock, (struct sockaddr *) &echoServAddr, sizeof(echoServAddr)) < 0)
        return -1;
    return sock;
}

EXPORT void change_usleep_const(int new_constant)
{usleep_const = new_constant;}

EXPORT void cnt_read_chg(char new_constant)
{cnt_read = new_constant;}
