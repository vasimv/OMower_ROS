// Gateway/proxy service for OMower
// $Id$

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <time.h>
#include <errno.h>
#include <string.h>
#include <unistd.h>
#include <netdb.h>
#include <arpa/inet.h> 
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <fcntl.h>
#include <termios.h>
#include <poll.h>

#include <string>
using namespace std;

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/UInt8MultiArray.h"

#define USE_RTK

// Listen port for pfodApp/modbus
int listenPort = 8991;

// RTKLIB's output socket
char *rtkAddr = "127.0.0.1";
int rtkPort = 8990;

#define MODBUS_MIN_DIFF 10
#define MODBUS_MIN_DELAY 200

#define STATUS_NONE 0
#define STATUS_LOG 1
#define STATUS_MODBUS 2

long countRtk = 0;
long countSent = 0;
long countRecv = 0;
long countCycles = 0;

FILE *flog;

ros::Subscriber subCmdOut;
ros::Subscriber subDebug;
ros::Publisher pubCmdIn;

class buffer {
  public:
    uint8_t *buf;
    int sizeBytes;
    
    void clear();
    int checkBuf();
    int pushBuf(uint8_t *p, int size);
    int pushBuf(uint8_t c);
    int pushBuf32(int32_t val);
    int pushBuf32(uint32_t val);
    int recvSock(int sock);
    int recvSock(int sock, int size);
    int sendSock(int sock);
    int sendROS(ros::Publisher pub);
    int remove(int size);
    buffer(int size);
    buffer();
  private:
    uint8_t *pBuf;
};

class circBuffer {
  public:
    uint8_t *buf;
    int sizeBytes;

    void clear();
    uint8_t fetch();
    int checkBuf();
    int recvSock(int sock);
    int pushBuf(uint8_t byte);
    int pushBuf(uint8_t *p, int size);
    circBuffer(int size);

  private:
    uint8_t *head;
    uint8_t *tail;
};

circBuffer::circBuffer(int size) {
  buf = (uint8_t *) malloc(size);
  sizeBytes = size;
  clear();
}

void circBuffer::clear() {
  head = tail = buf;
}

uint8_t circBuffer::fetch() {
  uint8_t c;

  if (head != tail) {
    c = *head;
    head++;
    if (head >= (buf + sizeBytes))
      head = buf;
    return c;
  }
  return 0;
}

int circBuffer::checkBuf() {
  if (tail >= head)
    return tail - head;
  return (tail - buf) + (buf + sizeBytes - head);
}

int circBuffer::pushBuf(uint8_t c) {
  if (checkBuf() < sizeBytes) {
    *tail = c;
    tail++;
    if (tail >= (buf + sizeBytes))
      tail = buf;
    return 1;
  }
  return 0;
}

int circBuffer::pushBuf(uint8_t *p, int size) {
  int i = 0;

  for (i = 0; i < size; i++)
    if (pushBuf(p[i]) == 0)
      return i;
  return size;
}

int circBuffer::recvSock(int sock) {
  int res = 0;
  int avail;

  if (checkBuf() < sizeBytes) {
    if (head <= tail)
      avail = sizeBytes - (tail - buf);
    else
      avail = head - tail - 1;
    res = read(sock, tail, avail);
    if (res > 0) {
      tail = (tail + res);
      if (tail >= (buf + sizeBytes))
        tail = buf;
    }
  }
  return res;
}

buffer::buffer() {
  buf = NULL;
  sizeBytes = 0;
}

buffer::buffer(int size) {
  buf = (uint8_t *) malloc(size);
  sizeBytes = size;
  clear();
}

void buffer::clear() {
  pBuf = buf;
}

int buffer::checkBuf() {
  return pBuf - buf;
}

int buffer::pushBuf(uint8_t c) {
  if ((pBuf - buf) < sizeBytes) {
    *pBuf = c;
    pBuf++;
    return 1;
  }
  return 0;
}

int buffer::pushBuf32(uint32_t val) {
  uint8_t *p = (uint8_t *) &val;

  pushBuf(p[3]);
  pushBuf(p[2]);
  pushBuf(p[1]);
  pushBuf(p[0]);
}

int buffer::pushBuf32(int32_t val) {
  uint8_t *p = (uint8_t *) &val;

  pushBuf(p[3]);
  pushBuf(p[2]);
  pushBuf(p[1]);
  pushBuf(p[0]);
}

int buffer::pushBuf(uint8_t *p, int size) {
  for (int i = 0; i < size; i++)
    if (pushBuf(p[i]) == 0)
      return i;
  return size; 
}

int buffer::recvSock(int sock, int size) {
  int res = 0;

  if ((sizeBytes - checkBuf()) >= size) {
    res = read(sock, pBuf, size); 
    if (res > 0)
      pBuf += res;
  }
  return res;
}

int buffer::recvSock(int sock) {
  return recvSock(sock, sizeBytes - checkBuf());
}

int buffer::sendSock(int sock) {
  int res = 0;

  if (pBuf != buf)
    res = write(sock, buf, pBuf - buf);
  return res;
}

int buffer::sendROS(ros::Publisher pub) {
  std_msgs::UInt8MultiArray msg;
  string outS("");
  uint8_t *p;
  int res = 0;

  if (pBuf != buf) {
    res = pBuf - buf;
    // Create string from buffer's content
    // Send to ROS topic
    msg.data.clear();
    msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
    msg.layout.dim[0].label = "";
    msg.layout.dim[0].stride = 1;
    msg.layout.dim[0].size = pBuf - buf;
    msg.layout.data_offset = 0;
    msg.data = std::vector<uint8_t>(buf, pBuf);
    pub.publish(msg);
  }
  return res;
}

int buffer::remove(int size) {
  int len;

  len = checkBuf();
  if (len < size)
    size = len;
  if (len == size) {
    clear();
    return size;
  }
  memmove(buf, buf + size, len - size);
  pBuf -= size;
  return size;
}

uint16_t modbusCrc(uint8_t *buf, int len);

struct timeval startTime;
struct timeval timeStamp;
struct timeval lastSentModbus;
struct timeval lastModbusUpdate;
struct timeval lastModbusAttempt;
struct timeval lastRtkAttempt;

buffer modbusIn(256);
buffer rtkIn(4096);
buffer incomingIn(4096);
circBuffer modbus(4096);
circBuffer rtk(4096);
circBuffer incoming(4096);

buffer outRtk(4096);
int flagRtkConfirmed = 0;
buffer outCommand(4096);
buffer outIncoming(4096);
buffer outModbus(128);
int flagCmdConfirmed = 0;

int sIncoming = -1;
int sModbus = -1;
int sRtk = -1;
int sListen = -1;

uint32_t varsModbus[2048];
int regStart = 0;
int regNum = 0;

int statusModbus = STATUS_NONE;
int flagVarsConfirmed = 1;

int diffMillis(struct timeval tv) {
  struct timeval cTv, rTv;

  gettimeofday(&cTv, NULL);
  timersub(&cTv, &tv, &rTv);
  return (rTv.tv_usec / 1000) + (rTv.tv_sec * 1000);
}

void sendReq(uint16_t start, uint16_t num) {
  uint16_t crc;

  outModbus.clear();
  outModbus.pushBuf(0xA5);
  outModbus.pushBuf(0x03);
  outModbus.pushBuf((start >> 8) & 0xff);
  outModbus.pushBuf(start & 0xff);
  outModbus.pushBuf((num >> 8) & 0xff);
  outModbus.pushBuf(num & 0xff);
  crc = modbusCrc(outModbus.buf, outModbus.checkBuf());
  outModbus.pushBuf((crc >> 8) & 0xff);
  outModbus.pushBuf(crc & 0xff);
  flagVarsConfirmed = 0;
}

void addCrc(buffer &buf) {
  uint16_t crc;

  crc = modbusCrc(buf.buf, buf.checkBuf());
  buf.pushBuf((crc >> 8) & 0xff);
  buf.pushBuf(crc & 0xff);
}

void sendCmdStop() {
  fprintf(stderr, "Issue Stop command\n");
  outModbus.clear();
  outModbus.pushBuf(0xA5);
  outModbus.pushBuf(0x10);
  outModbus.pushBuf(0x00);
  outModbus.pushBuf(0x04);
  outModbus.pushBuf(0);
  outModbus.pushBuf(2);
  outModbus.pushBuf(4);
  // CURR_COMMAND = CMD_STOP
  outModbus.pushBuf(0);
  outModbus.pushBuf(0);
  outModbus.pushBuf(0);
  outModbus.pushBuf(0);
  addCrc(outModbus);
}

// Powersave mode
void sendCmdPowersave(uint32_t secs) {
  fprintf(stderr, "Issue Powersave command\n");
  outModbus.clear();
  outModbus.pushBuf(0xA5);
  outModbus.pushBuf(0x10);
  outModbus.pushBuf(0x00);
  outModbus.pushBuf(0x02);
  outModbus.pushBuf(0);
  outModbus.pushBuf(4);
  outModbus.pushBuf(8);
  outModbus.pushBuf32(secs * 1000);
  // CURR_COMMAND = CMD_POWERSAVE
  outModbus.pushBuf32(21);
  addCrc(outModbus);
}

// Exiting powersave mode
void sendCmdNormal() {
  fprintf(stderr, "Issue Normal command\n");
  outModbus.clear();
  outModbus.pushBuf(0xA5);
  outModbus.pushBuf(0x10);
  outModbus.pushBuf(0x00);
  outModbus.pushBuf(0x04);
  outModbus.pushBuf(0);
  outModbus.pushBuf(2);
  outModbus.pushBuf(4);
  // CURR_COMMAND = CMD_NORMAL
  outModbus.pushBuf32(22);
  addCrc(outModbus);
}

void sendCmdShutdown(uint32_t secs) {
  fprintf(stderr, "Issue shutdown command (after %u seconds)\n", secs);
  outModbus.clear();
  outModbus.pushBuf(0xA5);
  outModbus.pushBuf(0x10);
  outModbus.pushBuf(0x00);
  outModbus.pushBuf(0x02);
  outModbus.pushBuf(0);
  outModbus.pushBuf(4);
  outModbus.pushBuf(8);
  outModbus.pushBuf32(secs * 1000);
  // CURR_COMMAND = CMD_SHUTDOWN
  outModbus.pushBuf32(23);
  addCrc(outModbus);
}

// Calculate CRC for modbus frame
uint16_t modbusCrc(uint8_t *buf, int len) {
  uint32_t tmp, tmp2;
  uint8_t Flag;
  uint16_t i, j;

  tmp = 0xFFFF;
  for (i = 0; i < len; i++) {
    tmp = tmp ^ buf[i];
    for (j = 1; j <= 8; j++) {
      Flag = tmp & 0x0001;
      tmp >>=1;
      if (Flag)
        tmp ^= 0xA001;
    }
  }
  tmp2 = tmp >> 8;
  tmp = (tmp << 8) | tmp2;
  tmp &= 0xFFFF;
  return (uint16_t) tmp;
}

void nonBlock(int fd) {
  int flags;

  flags = fcntl(fd, F_GETFL, 0);
  fcntl(fd, F_SETFL, flags | O_NONBLOCK);
}

// New connection (returns socket)
int createConn(char *addr, int port) {
  int sockfd;
  struct hostent *he;
  struct sockaddr_in their_addr; /* connector's address information */

  if ((he=gethostbyname(addr)) == NULL) {  /* get the host info */
    herror("gethostbyname");
    return -1;
  }

  if ((sockfd = socket(AF_INET, SOCK_STREAM, 0)) == -1) {
    perror("socket");
    return -1;
  }

  bzero(&their_addr, sizeof(their_addr));
  their_addr.sin_family = AF_INET;
  their_addr.sin_port = htons(port);
  their_addr.sin_addr = *((struct in_addr *) he->h_addr);
  bzero(&(their_addr.sin_zero), 8);

  if (connect(sockfd, (struct sockaddr *) &their_addr, sizeof(struct sockaddr)) == -1) {
    close(sockfd);
    perror("connect");
    return -1;
  }
  return sockfd;
}

// Close socket
void closeSock(int sock) {
  if (sock <= 0)
    return;
  close(sock);
  fprintf(stderr, "Closing socket fd %d\n", sock);
}

// Create listen socket
void openListen() {
  int opt = 1;
  struct sockaddr_in address;
  int sock = -1;

  fprintf(stderr, "Opening listen socket\n");
  while (sock < 0) {
    // create socket
    if ((sock = socket(AF_INET , SOCK_STREAM , 0)) <= 0) {
      perror("socket failed");
      sock = -1;
      continue;
    }

    // set master socket to allow multiple connections
    if (setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, (char *)&opt, sizeof(opt)) < 0) {
      closeSock(sock);
      sock = -1;
      perror("setsockopt");
      continue;
    }

    // Set type of socket created
    bzero(&address, sizeof(address));
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = INADDR_ANY;
    address.sin_port = htons(listenPort);

    // bind the socket
    if (bind(sock, (struct sockaddr *)&address, sizeof(address)) < 0) {
      perror("bind failed");
      closeSock(sock);
      sock = -1;  
    }
    listen(sock, 5);
  }
  sListen = sock;
  fprintf(stderr, "Listen socked fd %d\n", sock);
}

// Accept incoming connection
void acceptConn() {
  struct sockaddr_in address;
  int addrlen = sizeof(address);

  if (sIncoming >= 0)
    closeSock(sIncoming);
  bzero(&address, sizeof(address));
  sIncoming = accept(sListen, (struct sockaddr *) &address, (socklen_t *) &addrlen);
  if (sIncoming > 0) {
    fprintf(stderr, "New connection , socket fd is %d , ip is : %s , port : %d \n" , sIncoming , inet_ntoa(address.sin_addr) , ntohs(address.sin_port));
    nonBlock(sIncoming);
  } else
    sIncoming = -1;
}

// Process modbus packet from OMower
void processModbus() {
  uint16_t crc;
  uint16_t start;
  int len;

  // Wrong function code
  if ((modbusIn.buf[1] != 0x10) && (modbusIn.buf[1] != 0x03))
    return;
  countRecv++;
  // Verify checksum
  len = modbusIn.checkBuf();
  crc = (uint16_t) modbusIn.buf[len - 1] | ((uint16_t) modbusIn.buf[len - 2] << 8);
  if (crc != modbusCrc(modbusIn.buf, len - 2))
    return;
  // fprintf(stderr, "processModbus %d\n", len);
  switch (modbusIn.buf[1]) {
    case 0x03:
      // Check packet length
      if (modbusIn.buf[2] != (len - 5))
        return;
      if ((len - 5) != (regNum * 2))
        return;
      memcpy(varsModbus + regStart * 2, modbusIn.buf + 3, regNum * 2);
      flagVarsConfirmed = 1;
      break;
    case 0x10:
      if (len != 8)
        return;
      start = ((uint16_t) modbusIn.buf[2] << 8) | (uint16_t) modbusIn.buf[3];
      if (start == 44)
        flagRtkConfirmed = 1;
      else
        flagCmdConfirmed = 1;
      break;
  }
}

// Check for full modbus packet
int checkFullModbus() {
  int len;

  // fprintf(stderr, "checkFullModbus %02x %02x %d\n", modbusIn.buf[0], modbusIn.buf[1], modbusIn.checkBuf());
  // Write multiple registers ack
  if ((modbusIn.buf[1] == 0x10) && (modbusIn.checkBuf() >= 8))
    return 1;
  if (modbusIn.buf[1] == 0x03) {
    len = modbusIn.buf[2];
    if (modbusIn.checkBuf() >= (len + 5))
      return 1;
  }
  if ((modbusIn.buf[1] & 0x80) && (modbusIn.checkBuf() >= 5))
    return 1;
  return 0;
}

// Check incoming bytes from OMower (modbus or pfodApp/logs)
void checkModbus() {
  int len = modbus.checkBuf();
  uint8_t c;
  int i;

  for (i = 0; i < len; i++) {
    c = modbus.fetch();
    // fprintf(stderr, "c %02X %d\n", c, statusModbus);
    switch (statusModbus) {
      // First char
      case STATUS_NONE:
        if (c == 0xA5) {
          modbusIn.clear();
          statusModbus = STATUS_MODBUS;
          modbusIn.pushBuf(c);
        } else {
          if ((c != '}') && (c != '\n'))
            statusModbus = STATUS_LOG;
          outIncoming.pushBuf(c);
        }
        break;

      // pfodApp/log data
      case STATUS_LOG:
        // Check for end of line of pfodApp data
        if (c == 0xA5) {
          modbusIn.clear();
          modbusIn.pushBuf(c);
          statusModbus = STATUS_MODBUS;
        } else {
          outIncoming.pushBuf(c);
          if ((c == '}') || (c == '\n')) {
            statusModbus = STATUS_NONE;
          }
        }
        break;

      // modbus data
      case STATUS_MODBUS:
        if (modbusIn.checkBuf() >= modbusIn.sizeBytes) {
          if (c == 0xA5) {
            statusModbus = STATUS_MODBUS;
            modbusIn.clear();
            modbusIn.pushBuf(c);
          } else {
            if ((c != '}') && (c != '\n')) {
              modbusIn.clear();
              statusModbus = STATUS_LOG;
            }
            outIncoming.pushBuf(c);
          }
          break;
        }
        modbusIn.pushBuf(c);
        if (checkFullModbus()) {
          processModbus();
          modbusIn.clear();
          statusModbus = STATUS_NONE;
        }
        break;
    }
  }
}

// Receive from command socket
void receiveIncoming() {
  int res;

  if (incomingIn.checkBuf() >= incomingIn.sizeBytes)
    incomingIn.clear();
  res = incomingIn.recvSock(sIncoming);
  if (res > 0) {
    if (memchr(incomingIn.buf, '\n', incomingIn.checkBuf()) || memchr(incomingIn.buf, '}', incomingIn.checkBuf())) {
      outCommand.pushBuf(incomingIn.buf, incomingIn.checkBuf());
      incomingIn.clear();
    }
  }
  if (res == 0) {
    closeSock(sIncoming);
    sIncoming = -1;
  }
}

// Send to command socket
void sendIncoming() {
  int res;

  res = outIncoming.sendSock(sIncoming);
  if (res > 0)
    outIncoming.remove(res);
}

// Parse RTK message
void parseRtk() {
  int32_t lat, lon;
  int32_t a_lat, a_lon, b_lat, b_lon;
  int week, secweek;
  time_t gtime;
  int fix, numsats;
  struct tm *st;
  int year, month, mday, hour, minute, second;
  int res;
  uint16_t crc;

  res = sscanf((char *) rtkIn.buf, "%d/%d/%d %d:%d:%d.%*d %d.%d %d.%d %*f %d %d %*f %*f %*f %*f %*f %*f %*f %*f", &year, &month, &mday, &hour, &minute, &second, &a_lat, &b_lat, &a_lon, &b_lon, &fix, &numsats);
  if (res != 12)
    return;
  if (a_lat < 0)
    lat = a_lat * 10000000 - b_lat / 100;
  else
    lat = a_lat * 10000000 + b_lat / 100;
  if (a_lon < 0)
    lon = a_lon * 10000000 - b_lon / 100;
  else
    lon = a_lon * 10000000 + b_lon / 100;
  gtime = week * 604800 + secweek + 315964784;
  st = gmtime(&gtime);
  if (fix == 1)
    numsats += 128;
//  fprintf(stderr, "coord received: %d %d (%d %d), time %lu (%d-%d-%d %d:%d:%d)\n", lat, lon, fix, numsats, gtime, year, month, mday, hour, minute, second);
  countRtk++;

#ifdef RTK_USE_MODBUS
  // Create packet to set GPS coordinates on OMower
  outRtk.clear();
  outRtk.pushBuf(0xA5);
  outRtk.pushBuf(0x10);
  outRtk.pushBuf(0);
  outRtk.pushBuf(44);
  outRtk.pushBuf(0);
  outRtk.pushBuf(6);
  outRtk.pushBuf(12);
  outRtk.pushBuf(*(((uint8_t *) &lat) + 3));
  outRtk.pushBuf(*(((uint8_t *) &lat) + 2));
  outRtk.pushBuf(*(((uint8_t *) &lat) + 1));
  outRtk.pushBuf(*(((uint8_t *) &lat) + 0));
  outRtk.pushBuf(*(((uint8_t *) &lon) + 3));
  outRtk.pushBuf(*(((uint8_t *) &lon) + 2));
  outRtk.pushBuf(*(((uint8_t *) &lon) + 1));
  outRtk.pushBuf(*(((uint8_t *) &lon) + 0));
  outRtk.pushBuf(0);
  outRtk.pushBuf(0);
  outRtk.pushBuf(0);
  outRtk.pushBuf(numsats);
  crc = modbusCrc(outRtk.buf, outRtk.checkBuf());
  outRtk.pushBuf((crc >> 8) & 0xff);
  outRtk.pushBuf(crc & 0xff);
#else
  char out[256];

  snprintf(out, sizeof(out) - 1, "{g00`%d,%d,%d,%d,%d,%d,%d,%d,%d}\n",
           lat, lon, numsats, st->tm_year + 1900, st->tm_mon, st->tm_mday, st->tm_hour, st->tm_min, st->tm_sec);
  outCommand.pushBuf((uint8_t *) out, strlen(out));
#endif
}

// Open RTK connection
void openRtk() {
  int sock;

  if (diffMillis(lastRtkAttempt) > 2000) {
    fprintf(stderr, "Opening RTK connection\n");
    gettimeofday(&lastRtkAttempt, NULL);
    sock = createConn(rtkAddr, rtkPort);
    if (sock >= 0) {
      sRtk = sock;
      nonBlock(sock);
      return;
    }
  }
}

// Receive from RTK socket
void receiveRtk() {
  int res;

  flagRtkConfirmed = 0;
  res = rtkIn.recvSock(sRtk);
  if (res > 0) {
    if (rtkIn.checkBuf() >= rtkIn.sizeBytes)
      rtkIn.clear();
    if (memchr(rtkIn.buf, '\n', rtkIn.checkBuf())) {
      parseRtk();
      rtkIn.clear();
    }
  }
  if (res == 0) {
    closeSock(sRtk);
    sRtk = -1;
  }
}

// Process input from OMower commands output stream
void receiveModbus() {
  int res;

  if (modbus.checkBuf() == 0)
    return;

  if (modbus.checkBuf() >= modbus.sizeBytes)
    modbus.clear();

  checkModbus();
  gettimeofday(&lastModbusUpdate, NULL);
}

// Receive callback for OMower commands output stream
void cbCmdOut(const std_msgs::UInt8MultiArray::ConstPtr &msg) {
  if (modbus.checkBuf() >= modbus.sizeBytes)
    modbus.clear();
  modbus.pushBuf((uint8_t *) msg->data.data(), msg->data.size());
  receiveModbus();
}

void cbDebug(const std_msgs::String::ConstPtr &msg) {
  fprintf(stderr, "%s", msg->data.c_str());
}

// Send to Modbus socket
void sendModbus() {
  int res;
  buffer *curBuf = NULL;

  if (outRtk.checkBuf() > 0)
    curBuf = &outRtk;
  else {
    if (outModbus.checkBuf() > 0)
      curBuf = &outModbus;
    else {
      if (outCommand.checkBuf() > 0)
        curBuf = &outCommand;
    }
  }
  if (curBuf) {
    res = curBuf->sendROS(pubCmdIn);
    if (res >= curBuf->checkBuf()) {
      countSent++;
      curBuf->clear();
      gettimeofday(&lastSentModbus, NULL);
    } else {
      curBuf->remove(res);
      // Reset timer counter to prevent waiting for modbus pause
      lastSentModbus.tv_sec = startTime.tv_sec;
      lastSentModbus.tv_usec = startTime.tv_usec;
    }
  }
}

struct timeval timeReturn;
struct timeval timeBattery;
struct timeval timeLast;

// Poll sockets/files and receive/send all stuff
void commPoll() {
  struct pollfd fds[3];
  int i, rc;

  memset(fds, 0, sizeof(fds));

  if (sIncoming >= 0) {
    fds[0].fd = sIncoming;
    fds[0].events = POLLIN;
    if (outIncoming.checkBuf() > 0)
      fds[0].events |= POLLOUT;
  } else
    fds[0].fd = -1;

  if (sRtk >= 0) {
    fds[1].fd = sRtk;
    fds[1].events = POLLIN | POLLNVAL | POLLERR;
  } else
    fds[1].fd = -1;

  fds[2].fd = sListen;
  fds[2].events = POLLIN;

  countCycles++;
  if (countCycles % 10000 == 0)
    fprintf(stderr, "cycles: %ld, rtk: %ld, sent: %ld, recv: %ld\n",
            countCycles, countRtk, countSent, countRecv);
  rc = poll(fds, 3, 1);
  if (rc <= 0)
    return;

  // Check listening socket events
  if (fds[2].revents & POLLIN) {
    fprintf(stderr, "Incoming connection\n");
    closeSock(sIncoming);
    sIncoming = -1;
    acceptConn();
  }
  // Check incoming commands socket
  if (fds[0].revents & POLLIN)
    receiveIncoming();
  if (fds[0].revents & POLLOUT)
    sendIncoming();
  if ((fds[0].revents & POLLNVAL) || (fds[0].revents & POLLERR) || (fds[0].revents & POLLHUP)) {
    closeSock(sIncoming);
    sIncoming = -1;
    incomingIn.clear();
    incoming.clear();
  }

#ifdef USE_RTK
  // Check RTKLIB socket
  if (fds[1].revents & POLLIN)
    receiveRtk();
  if ((sRtk == -1) || (fds[1].revents & POLLNVAL) || (fds[1].revents & POLLERR) || (fds[1].revents & POLLHUP)) {
    closeSock(sRtk);
    openRtk();
  }
#endif
}

int main(int argc, char **argv) {
  char name[256];
  int secs;

  ros::init(argc, argv, "omower_powersave");
  ros::NodeHandle nh;
  pubCmdIn = nh.advertise<std_msgs::UInt8MultiArray>("cmdIn", 1000);

  secs = atoi(argv[1]);
  fprintf(stderr, "POWERSAVE for %d secs\n", secs);
  sleep(30);

  outModbus.clear();
  sendCmdPowersave(secs);
  int res = outModbus.sendROS(pubCmdIn);
  ros::spinOnce();
  fprintf(stderr, "sent POWERSAVE (%d)\n", res);
  sleep(2);
}

