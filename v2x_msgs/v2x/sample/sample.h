/*
 * UPER Based client application sample code
 *
 * Copyright (c) 2022, CEST
 *
 */

#include "j2735.h"
 
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <fcntl.h>

#define DEBUG_OPTION
 
#define OBU_RECEIVE_BUFFER_SIZE 1500
#define MAX_UPER_SIZE 1400

#define PVD_INTERVAL 200 //msec, 220616에 변경
// #define PVD_INTERVAL 1000 //msec

#define BSM_INTERVAL 100 //msec
 
// OBU와 연결할 차량시스템의 기본 정보
unsigned char packetSeq = 0; // TCP 전송 Header 내 Sequence Number 값 
unsigned char clientDeviceId[3] = {0x01,0x02,0x03}; // TCP 전송 Header 내 Device ID
unsigned char temporaryId[4] = {0x00,0x01,0x02,0x03}; // BSM, PVD 전송 시 메시지 내 Temporary ID


#include <time.h>

/**
 * @brief Get the clock time object
 * 
 * @return unsigned long long 
 */
unsigned long long get_clock_time();

/**
 * @brief CEST V2X OBU와 TCP 통신으로 데이터 송수신을 위한 TCP 헤더
 * 
 */
struct CestObuUperPacketHeader
{
    unsigned short messageType; // 메시지의 타입
    unsigned char seq;          // 메시지의 시퀀스
    unsigned short payloadLen;  // 페이로드 길이
    unsigned char deviceType;   // 장치 종류
    unsigned char deviceId[3];     // 장치 ID (MAC 하위 3바이트)
}__attribute__((packed));


/**
 * @brief CEST V2X OBU로 요청한 TX_WAVE_UPER에 대한 전송 결과 메시지
 * 
 */
struct TxWaveUperResultPayload
{
   unsigned char txWaveUperSeq;
   unsigned char resultCode;
   unsigned char size;
}__attribute__((packed)); 
 

/**
 * @brief TCP Client 소켓을 생성 후, OBU로 접속
 *
 * @param ip    OBU의 IP 주소 "x.x.x.x"
 * @param port  OBU의 TCP 포트 번호 
 * @return int  TCP 커넥션 유무 (>0 : 연결되어 연결된 Socket을 반환, -1 : 연결 실패)
 */
int connect_obu_uper_tcp(char *ip, unsigned short port);


/**
 * @brief 접속된 OBU 소켓으로부터 RX_WAVE_UPER , TX_WAVE_UPER_RESULT 수신
 * 
 * @param sockFd 
 * @param buffer 
 * @param bufferSize 
 * @param storedSize 
 * @param uperBuffer 
 * @param uperBufferSize 
 * @param uperRes 
 * @return int 
 */
int receive_from_obu(int sockFd,
                     char *buffer,
                     unsigned short bufferSize,
                     int storedSize,
                     char *uperBuffer,
                     unsigned short uperBufferSize,
                     int *uperRes);

/**
 * @brief TX_WAVE_UPER 메시지 전송 
 * 
 * @param sockFd OBU Client socket 
 * @param uper   UPER 인코딩된 MessageFrame
 * @param uperLength  UPER 인코딩된 MessageFrame의 Byte 길이
 * @return int  -1 : 전송 실패 , >0 전송된 바이트 수
 */
int request_tx_wave_obu(int sockFd, char *uper, unsigned short uperLength);

/**
 * @brief PVD 메시지 생성 및 WAVE 전송 
 * 
 * @param sockFd   OBU Client socket 
 * @param time     PVD 전송 Timer Pointer
 * @return int     -1 : 전송 실패
 */
int tx_v2i_pvd(int sockFd, unsigned long long *time);

/**
 * @brief BSM 메시지 생성 및 WAVE 전송
 * 
 * @param sockFd  OBU Client socket 
 * @param time    BSM 전송 Timer Pointer
 * @return int    -1 : 전송 실패
 */
int tx_v2v_bsm(int sockFd, unsigned long long *time);

