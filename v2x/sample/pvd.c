/*
 * UPER Based client application sample code
 *
 * Copyright (c) 2022, CEST
 *
 */

#include "j2735.h"
#include <time.h>
#include <stdio.h>
#include <fcntl.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <string.h>

#define MAX_BUFFER 600

typedef struct LabviewData
{
    long latitude;
    long longitude;
    long heading;
    long velocity;
    long gear;
}LabveiwData_t;

LabveiwData_t socket_server();

int fill_j2735_pvd(MessageFrame_t *dst)
{   
    //utc 시간 구조체 정의
    time_t seconds = time(NULL);
    struct tm *now = gmtime(&seconds);
    // printf("[%04d/%02d/%02d] %02d:%02d:%02d\n", 1900 + now->tm_year, 
    //         now->tm_mon + 1, now->tm_mday, now->tm_hour, 
    //         now->tm_min, now->tm_sec);

    // ASN_STRUCT_RESET(asn_DEF_MessageFrame, dst);

    dst->messageId = 26; // J2735 표준문서 PDF 파일 참조 DE_DSRC_MessageID,  probeVehicleData DSRCmsgID ::= 26 -- PVD 
    dst->value.present = MessageFrame__value_PR_ProbeVehicleData; // MessageFrame::value choice (asn1c)

    ProbeVehicleData_t *ptrPvd = &dst->value.choice.ProbeVehicleData;

    ptrPvd->timeStamp = NULL; // OPTIONAL, not to use
    ptrPvd->segNum = NULL;    // OPTIONAL, not to use
    ptrPvd->regional = NULL;  // OPTIONAL, not to use

    ptrPvd->probeID = malloc(sizeof(struct VehicleIdent));
    ptrPvd->probeID->name = NULL;         // OPTIONAL, not to use
    ptrPvd->probeID->ownerCode = NULL;    // OPTIONAL, not to use
    ptrPvd->probeID->vehicleClass = NULL; // OPTIONAL, not to use
    ptrPvd->probeID->vin = NULL;          // OPTIONAL, not to use
    ptrPvd->probeID->vehicleType = NULL;  // OPTIONAL, not to use
    ptrPvd->probeID->id = malloc(sizeof (struct VehicleID));
    ptrPvd->probeID->id->present = VehicleID_PR_entityID;   
    ptrPvd->probeID->id->present = VehicleID_PR_entityID;
    ptrPvd->probeID->id->choice.entityID.buf = (unsigned char *)malloc(4);
    ptrPvd->probeID->id->choice.entityID.size = 4;
    ptrPvd->probeID->id->choice.entityID.buf[0] = 0xCE;      // (INPUT) <---- 할당된 대학별 ID 입력
    ptrPvd->probeID->id->choice.entityID.buf[1] = 0x24;      // (INPUT) <---- 할당된 대학별 ID 입력
    ptrPvd->probeID->id->choice.entityID.buf[2] = 0x67;      // (INPUT) <---- 할당된 대학별 ID 입력
    ptrPvd->probeID->id->choice.entityID.buf[3] = 0x01;      // (INPUT) <---- 할당된 대학별 ID 입력
  
    //StartVector : PVD를 전송할 시점을 기준의 시간과 차량의 위치, 이동상태 값을 반영 
    //메모리 할당 
    ptrPvd->startVector.utcTime = malloc(sizeof(struct DDateTime));  
    ptrPvd->startVector.utcTime->year = malloc(sizeof(DYear_t));
    ptrPvd->startVector.utcTime->month = malloc(sizeof(DMonth_t)); 
    ptrPvd->startVector.utcTime->day = malloc(sizeof(DDay_t)); 
    ptrPvd->startVector.utcTime->hour = malloc(sizeof(DHour_t)); 
    ptrPvd->startVector.utcTime->minute = malloc(sizeof(DMinute_t)); 
    ptrPvd->startVector.utcTime->second = malloc(sizeof(DSecond_t)); 
    ptrPvd->startVector.utcTime->offset = NULL; // OPTIONAL, not to use

    //값 할당
    *ptrPvd->startVector.utcTime->year = now->tm_year + 1900; // (INPUT) <--------------- 현재 UTC 시간 입력 (년도)
    *ptrPvd->startVector.utcTime->month = now->tm_mon + 1;  // (INPUT) <--------------- 현재 UTC 시간 입력 (년도)
    *ptrPvd->startVector.utcTime->day = now->tm_mday;     // (INPUT) <--------------- 현재 UTC 시간 입력 (년도)
    *ptrPvd->startVector.utcTime->hour = now->tm_hour;    // (INPUT) <--------------- 현재 UTC 시간 입력 (년도)
    *ptrPvd->startVector.utcTime->minute = now->tm_min;  // (INPUT) <--------------- 현재 UTC 시간 입력 (년도)
    *ptrPvd->startVector.utcTime->second = (now->tm_sec)*1000;  // (INPUT) <--------------- 현재 UTC 시간 입력 (년도)

    ptrPvd->startVector.elevation = malloc(sizeof(DSRC_Elevation_t));
    ptrPvd->startVector.heading = malloc(sizeof(Heading_t));
    ptrPvd->startVector.speed = malloc(sizeof(struct TransmissionAndSpeed));
    ptrPvd->startVector.posAccuracy = NULL;     // OPTIONAL, not to use
    ptrPvd->startVector.posConfidence = NULL;   // OPTIONAL, not to use
    ptrPvd->startVector.timeConfidence = NULL;  // OPTIONAL, not to use
    ptrPvd->startVector.speedConfidence = NULL; // OPTIONAL, not to use

    LabveiwData_t data = socket_server();
    ptrPvd->startVector.Long = data.longitude;                // (INPUT) <--------------- 현재 차량의 위치 (위도) (Longitude, DD 좌표계)
    ptrPvd->startVector.lat = data.latitude; 
    *ptrPvd->startVector.elevation = 0;          // (INPUT) <--------------- 현재 차량의 위치 (고도) (Elevation)   
    *ptrPvd->startVector.heading = data.heading;            // (INPUT) <--------------- 현재 차량의 주행 방향 (북쪽 0도)           
    ptrPvd->startVector.speed->speed = data.velocity;        // (INPUT) <--------------- 현재 차량의 속도        
    ptrPvd->startVector.speed->transmisson = data.gear;  // (INPUT) <--------------- 현재 차량의 변속기 상태          
 
    // ptrPvd->startVector.Long = 1;                // (INPUT) <--------------- 현재 차량의 위치 (위도) (Longitude, DD 좌표계)
    // ptrPvd->startVector.lat = 1; 
    // *ptrPvd->startVector.elevation = 1;          // (INPUT) <--------------- 현재 차량의 위치 (고도) (Elevation)   
    // *ptrPvd->startVector.heading = 1;            // (INPUT) <--------------- 현재 차량의 주행 방향 (북쪽 0도)           
    // ptrPvd->startVector.speed->speed = 1;        // (INPUT) <--------------- 현재 차량의 속도        
    // ptrPvd->startVector.speed->transmisson = 1;  // (INPUT) <--------------- 현재 차량의 변속기 상태          

    ptrPvd->vehicleType.hpmsType = malloc(sizeof(VehicleType_t));
    ptrPvd->vehicleType.keyType = NULL;       // OPTIONAL, not to use
    ptrPvd->vehicleType.fuelType = NULL;      // OPTIONAL, not to use
    ptrPvd->vehicleType.iso3883 = NULL;       // OPTIONAL, not to use
    ptrPvd->vehicleType.regional = NULL;      // OPTIONAL, not to use
    ptrPvd->vehicleType.responderType = NULL; // OPTIONAL, not to use
    ptrPvd->vehicleType.responseEquip = NULL; // OPTIONAL, not to use
    ptrPvd->vehicleType.role = NULL;          // OPTIONAL, not to use
    ptrPvd->vehicleType.vehicleType = NULL;   // OPTIONAL, not to use
    *ptrPvd->vehicleType.hpmsType = VehicleType_car; 
 
    // PVD 전송 직전에 전송한 PVD startVector 시간, 위치, 이동상태를 입력 
    ptrPvd->snapshots.list.count = 1; 
    ptrPvd->snapshots.list.array = malloc(sizeof(struct Snapshot *));
    ptrPvd->snapshots.list.array[0] = malloc(sizeof(struct Snapshot));
    struct Snapshot *ptrSnapshot = ptrPvd->snapshots.list.array[0]; 

    ptrSnapshot->thePosition.utcTime = malloc(sizeof(struct DDateTime));
    ptrSnapshot->thePosition.utcTime->year = malloc(sizeof(DYear_t));
    ptrSnapshot->thePosition.utcTime->month = malloc(sizeof(DMonth_t));
    ptrSnapshot->thePosition.utcTime->day = malloc(sizeof(DDay_t));
    ptrSnapshot->thePosition.utcTime->hour = malloc(sizeof(DHour_t));
    ptrSnapshot->thePosition.utcTime->minute = malloc(sizeof(DMinute_t));
    ptrSnapshot->thePosition.utcTime->second = malloc(sizeof(DSecond_t));
    ptrSnapshot->thePosition.utcTime->offset = NULL; // OPTIONAL, not to use

    ptrSnapshot->thePosition.elevation = malloc(sizeof(DSRC_Elevation_t));
    ptrSnapshot->thePosition.speed = malloc(sizeof(struct TransmissionAndSpeed));
    ptrSnapshot->thePosition.heading = malloc(sizeof(Heading_t));
    ptrSnapshot->thePosition.posAccuracy = NULL;     // OPTIONAL, not to use
    ptrSnapshot->thePosition.posConfidence = NULL;   // OPTIONAL, not to use
    ptrSnapshot->thePosition.timeConfidence = NULL;  // OPTIONAL, not to use
    ptrSnapshot->thePosition.speedConfidence = NULL; // OPTIONAL, not to use

    *ptrSnapshot->thePosition.utcTime->year = *ptrPvd->startVector.utcTime->year;       // (INPUT) <--------------- 직전 전송한 PVD의 UTC 시간 입력 (년도)
    *ptrSnapshot->thePosition.utcTime->month = *ptrPvd->startVector.utcTime->month;         // (INPUT) <--------------- 직전 전송한 PVD의 UTC 시간 입력 (월)
    *ptrSnapshot->thePosition.utcTime->day = *ptrPvd->startVector.utcTime->day;           // (INPUT) <--------------- 직전 전송한 PVD의 UTC 시간 입력 (일)
    *ptrSnapshot->thePosition.utcTime->hour = *ptrPvd->startVector.utcTime->hour;          // (INPUT) <--------------- 직전 전송한 PVD의 UTC 시간 입력 (시)
    *ptrSnapshot->thePosition.utcTime->minute = *ptrPvd->startVector.utcTime->minute;        // (INPUT) <--------------- 직전 전송한 PVD의 UTC 시간 입력 (분)
    *ptrSnapshot->thePosition.utcTime->second = *ptrPvd->startVector.utcTime->second;        // (INPUT) <--------------- 직전 전송한 PVD의 UTC 시간 입력 (초)
    
    ptrSnapshot->thePosition.lat = ptrPvd->startVector.lat;                // (INPUT) <--------------- 현재 차량의 위치 (위도) (Longitude, DD 좌표계)
    ptrSnapshot->thePosition.Long = ptrPvd->startVector.Long;               // (INPUT) <--------------- 현재 차량의 위치 (경도) (Latitude,  DD 좌표계) 
    *ptrSnapshot->thePosition.elevation = *ptrPvd->startVector.elevation;         // (INPUT) <--------------- 현재 차량의 위치 (고도) (Elevation)   
    *ptrSnapshot->thePosition.heading = *ptrPvd->startVector.heading;           // (INPUT) <--------------- 현재 차량의 주행 방향 (북쪽 0도)               
    ptrSnapshot->thePosition.speed->speed = ptrPvd->startVector.speed->speed;       // (INPUT) <-------------- -현재 차량의 속도                  
    ptrSnapshot->thePosition.speed->transmisson = ptrPvd->startVector.speed->transmisson; // (INPUT) <--------------- 현재 차량의 변속기 상태   

    // *ptrSnapshot->thePosition.utcTime->year = 0;       // (INPUT) <--------------- 직전 전송한 PVD의 UTC 시간 입력 (년도)
    // *ptrSnapshot->thePosition.utcTime->month = 0;         // (INPUT) <--------------- 직전 전송한 PVD의 UTC 시간 입력 (월)
    // *ptrSnapshot->thePosition.utcTime->day = 0;           // (INPUT) <--------------- 직전 전송한 PVD의 UTC 시간 입력 (일)
    // *ptrSnapshot->thePosition.utcTime->hour = 0;          // (INPUT) <--------------- 직전 전송한 PVD의 UTC 시간 입력 (시)
    // *ptrSnapshot->thePosition.utcTime->minute = 0;        // (INPUT) <--------------- 직전 전송한 PVD의 UTC 시간 입력 (분)
    // *ptrSnapshot->thePosition.utcTime->second = 0;        // (INPUT) <--------------- 직전 전송한 PVD의 UTC 시간 입력 (초)
    
    // ptrSnapshot->thePosition.lat = 0;                // (INPUT) <--------------- 현재 차량의 위치 (위도) (Longitude, DD 좌표계)
    // ptrSnapshot->thePosition.Long = 0;               // (INPUT) <--------------- 현재 차량의 위치 (경도) (Latitude,  DD 좌표계) 
    // *ptrSnapshot->thePosition.elevation = 0;         // (INPUT) <--------------- 현재 차량의 위치 (고도) (Elevation)   
    // *ptrSnapshot->thePosition.heading = 0;           // (INPUT) <--------------- 현재 차량의 주행 방향 (북쪽 0도)               
    // ptrSnapshot->thePosition.speed->speed = 0;       // (INPUT) <-------------- -현재 차량의 속도                  
    // ptrSnapshot->thePosition.speed->transmisson = 0; // (INPUT) <--------------- 현재 차량의 변속기 상태         

    char a[4], b[4], c[4], d[4];
    sprintf(a, "%X", ptrPvd->probeID->id->choice.entityID.buf[0]);
    sprintf(b, "%X", ptrPvd->probeID->id->choice.entityID.buf[1]);
    sprintf(c, "%X", ptrPvd->probeID->id->choice.entityID.buf[2]);
    sprintf(d, "%X", ptrPvd->probeID->id->choice.entityID.buf[3]);

    //Interface
    printf(">> Parse J2735 : PVD\n");
    // printf("    probeID : %s:%s:%s:0%s\n", a,b,c,d);
    // printf("    startVector\n");
    // printf("     ㄴ utcTime\n");
    // printf("     \tㄴ year : %ld\n", *ptrPvd->startVector.utcTime->year);
    // printf("     \tㄴ month: %ld\n", *ptrPvd->startVector.utcTime->month);
    // printf("     \tㄴ day : %ld\n", *ptrPvd->startVector.utcTime->day);
    // printf("     \tㄴ hour : %ld\n", *ptrPvd->startVector.utcTime->hour);
    // printf("     \tㄴ minute : %ld\n", *ptrPvd->startVector.utcTime->minute);
    // printf("     \tㄴ sec : %ld\n", *ptrPvd->startVector.utcTime->second);
    // printf("     Long : %ld\n", ptrPvd->startVector.Long);
    // printf("     lat : %ld\n", ptrPvd->startVector.lat);
    // printf("     elevation : %ld\n", *ptrPvd->startVector.elevation);
    // printf("     heading : %ld\n", *ptrPvd->startVector.heading);
    // printf("     speed\n");
    // printf("      ㄴ speed : %ld\n", ptrPvd->startVector.speed->speed);
    // printf("      ㄴ transmission : %ld\n", ptrPvd->startVector.speed->transmisson);
    // printf("    vehicleType\n");
    // printf("\thpmsType : %ld\n", *ptrPvd->vehicleType.hpmsType);
    for (int i=0; i < ptrPvd->snapshots.list.count; i++){
    //     printf("    snapshots[%d]\n", i);
    //     printf("\tthePosition\n");
    //     printf("         ㄴ utcTime\n");
    //     printf("\t      ㄴ year : %ld\n", *ptrPvd->snapshots.list.array[i]->thePosition.utcTime->year);
    //     printf("\t      ㄴ month: %ld\n", *ptrPvd->snapshots.list.array[i]->thePosition.utcTime->month);
    //     printf("\t      ㄴ day : %ld\n", *ptrPvd->snapshots.list.array[i]->thePosition.utcTime->day);
    //     printf("\t      ㄴ hour : %ld\n", *ptrPvd->snapshots.list.array[i]->thePosition.utcTime->hour);
    //     printf("\t      ㄴ minute : %ld\n", *ptrPvd->snapshots.list.array[i]->thePosition.utcTime->minute);
    //     printf("\t      ㄴ sec : %ld\n", *ptrPvd->snapshots.list.array[i]->thePosition.utcTime->second);
    //     printf("     Long : %ld\n", ptrPvd->snapshots.list.array[i]->thePosition.Long);
    //     printf("     lat : %ld\n", ptrPvd->snapshots.list.array[i]->thePosition.lat);
    //     printf("     elevation : %ld\n", *ptrPvd->snapshots.list.array[i]->thePosition.elevation);
    //     printf("     heading : %ld\n", *ptrPvd->snapshots.list.array[i]->thePosition.heading);
    //     printf("     speed\n");
    //     printf("      ㄴ speed : %ld\n", ptrPvd->snapshots.list.array[i]->thePosition.speed->speed);
    //     printf("      ㄴ transmission : %ld\n\n", ptrPvd->snapshots.list.array[i]->thePosition.speed->transmisson);
    }
    return 0;
}

LabveiwData_t socket_server(){
    LabveiwData_t data;
    // int result[5];
    int sockFd = -1;
    int returnStatus;
    sockFd = socket(AF_INET, SOCK_DGRAM, 0);
    char buf[MAX_BUFFER];

    if (sockFd < 0){
        printf("ERROR : 랩뷰 데이터 수신 소켓 생성 에러\n");
    }
    else{printf("소켓 생성 완료\n");}

    struct sockaddr_in addr, udpClient; 
    memset(&addr, 0, sizeof(addr)); //메모리 초기화
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = inet_addr("127.0.0.1");
    addr.sin_port = htons(54110);

    int flag = fcntl(sockFd, F_GETFL, 0);
    fcntl(sockFd, F_SETFL, flag | O_NONBLOCK);

    if(bind(sockFd,(struct sockaddr*)&addr, sizeof(addr)) < 0){ //2번
        printf("bind error\n");
    }

    while(1){
        int addrlen = sizeof(udpClient);
        returnStatus = recvfrom(sockFd, buf, MAX_BUFFER, 0, (struct sockaddr*)&udpClient, &addrlen);
    if(returnStatus < 0){   
        continue;
        }
    else{
        // printf("수신 내용 : %s\n", buf);
        char *ptr = strtok(buf, ",");      // " " 공백 문자를 기준으로 문자열을 자름, 포인터 반환
        int k=0;
        while (ptr != NULL)               // 자른 문자열이 나오지 않을 때까지 반복
        {   
            // printf("%s\n", ptr);          // 자른 문자열 출력
            ptr = strtok(NULL, ",");      // 다음 문자열을 잘라서 포인터를 반환
            if(k==0){
                data.latitude = atol(ptr);
            }
            if(k==1){
                data.longitude = atol(ptr);
            }
            if(k==2){
                data.heading = atol(ptr);
            }
            if(k==3){
                data.velocity = atol(ptr);
            }
            if(k==4){
                data.gear = atol(ptr);
            }
            k++;
        }
        break;
        }
    }

    close(sockFd);
    return data;
}