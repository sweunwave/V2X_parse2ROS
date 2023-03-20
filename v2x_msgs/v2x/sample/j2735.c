/*
 * UPER Based client application sample code
 *
 * Copyright (c) 2022, CEST
 *
 */

#include <stdio.h>
#include "j2735.h"
#include <string.h>
#include <stdlib.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>

#define MAX_BUFFER 2048

void udp_sender(char buff[MAX_BUFFER], char* receiverIP, int receiverPORT){  //랩뷰는 50116, 파이썬은 50115
    int sockFd = -1;
    int returnStatus;
    sockFd = socket(AF_INET, SOCK_DGRAM, 0);
    char buf[MAX_BUFFER];

    if (sockFd < 0){
        printf("ERROR : V2X 데이터 송신 소켓 에러\n");
    }
    else{printf("소켓 생성 완료\n");}

    struct sockaddr_in udpSender; 
    memset(&udpSender, 0, sizeof(udpSender)); //메모리 초기화
    udpSender.sin_family = AF_INET;
    udpSender.sin_addr.s_addr = inet_addr(receiverIP);
    udpSender.sin_port = htons(receiverPORT);
    sendto(sockFd, buff, strlen(buff), 0, (struct sockaddr*)&udpSender, sizeof(udpSender));
    
    close(sockFd);
}

void print_hex(char *data, int len){
    printf("HEX[%d] : ",len);
    for(int i = 0 ; i < len ; i++){
        printf("%02X",(data[i] & 0xFF));
    }
    printf("\n");
}

int encode_j2735_uper(char *dst, unsigned short dstLen, MessageFrame_t *src)
{
    int res = -1;
    asn_enc_rval_t ret = uper_encode_to_buffer(&asn_DEF_MessageFrame,
                                               NULL,
                                               src,
                                               dst, dstLen);

    if (ret.encoded > 0){
        return ret.encoded; //  UPER Encoding Success
    }
    else
    { 
        if (ret.failed_type != NULL)
            printf("encoded error value name = %s\n", ret.failed_type->name);

        return -1; // UPER Encoding failed
    }
}

int decode_j2735_uper(MessageFrame_t *dst, char *src, int size){ 
  
    int res = -1;

    MessageFrame_t *ptrMsg = NULL; 

    asn_dec_rval_t ret = uper_decode(NULL,
                                     &asn_DEF_MessageFrame,
                                     (void **)&dst,
                                     src, size, 0, 0);

    if (ret.code != RC_OK){
        return res;
    }
    
    res = ret.consumed;
 
    // asn_fprint(stdout,&asn_DEF_MessageFrame,dst);

    parse_decoded_j2735(dst);

    return res;
}
 
int parse_decoded_j2735(MessageFrame_t *msg)
{ 
    switch(msg->messageId){
        case DSRC_ID_BSM:
            printf(">> Parse J2735 : BSM\n");
            break;
        case DSRC_ID_SPAT:
            printf(">> Parse J2735 : SPAT\n"); 
            parse_spat(&msg->value.choice.SPAT);
            break;  
        case DSRC_ID_MAP:
            printf(">> Parse J2735 : MAP\n");
            parse_map(&msg->value.choice.MapData);
            break;
    }
    return 0;
}

int parse_map(MapData_t *map){
    for (int i = 0; i < map->intersections->list.count; i++)
    {
        struct IntersectionGeometry *ptr= map->intersections->list.array[i]; 
        // MISSION : MAP 메시지에 포함된 IntersectionGeometry별 Intersection ID 추출
        //           Intersection 내 Ref Position을 기준으로 LaneSet의 Node 좌표 계산 및 출력
        //           Node로 연결된 차선의 Line 별 ID와 SignalGroupID를 출력 
        
    }
    return 0;

}
int parse_spat(SPAT_t *spat){
    char buff[MAX_BUFFER]="$42_V2X";
    char buf_intersections[2048], buf_states[1024], buf_stateTimeSpeed[128], buf_manueverAssistList[128];
    // printf("i 는 몇일까요오 : %d \n", spat->intersections.list.count); 
    for (int i = 0; i < spat->intersections.list.count; i++)
    {
        struct IntersectionState *ptr = spat->intersections.list.array[i];  
        // printf("j 는 몇일까요오 : %d \n", ptr->states.list.count); 
        printf("    name : '%s' \n", ptr->name->buf);
        printf("    intersection \
        \n     ㄴ region : %d \
        \n     ㄴ id : %ld\n", 0, ptr->id.id);
        printf("    revision : %ld\n", ptr->revision);
        printf("    status : %s\n", "33792(0x8400)");
        printf("    moy : %ld\n", *ptr->moy);
        printf("    timeStamp : %ld\n", *ptr->timeStamp);
        sprintf(buf_intersections, ",intersections[%d] %s %d %ld %ld %s %ld %ld", i, ptr->name->buf, 0,ptr->id.id, ptr->revision,"33792(0x8400)",*ptr->moy,*ptr->timeStamp);
        for (int j = 0; j < ptr->states.list.count; j++){
            printf("        states[%d]\n", j);
            printf("\t ㄴ movementName : %s\n", ptr->states.list.array[j]->movementName->buf);
            printf("\t ㄴ signalGruop : %ld\n", ptr->states.list.array[j]->signalGroup);
            printf("\t  state-time-speed\n");
            sprintf(buf_states, ":states[%d] %s %ld", j,ptr->states.list.array[j]->movementName->buf, ptr->states.list.array[j]->signalGroup);
            for(int k=0; k<ptr->states.list.array[j]->state_time_speed.list.count; k++){
                printf("\t   ㄴ eventState : %ld\n", ptr->states.list.array[j]->state_time_speed.list.array[k]->eventState);
                printf("\t   ㄴ timing_minEndTime : %ld\n", ptr->states.list.array[j]->state_time_speed.list.array[k]->timing->minEndTime);
                sprintf(buf_stateTimeSpeed, "-sts[%d] %ld %ld",k,ptr->states.list.array[j]->state_time_speed.list.array[k]->eventState,ptr->states.list.array[j]->state_time_speed.list.array[k]->timing->minEndTime);
                strcat(buf_states, buf_stateTimeSpeed); //state-time-speed 데이터 저장
            }
            printf("\t  maneuverAssistList\n");
            for(int q=0; q<ptr->states.list.array[j]->maneuverAssistList->list.count; q++){
                printf("\t   ㄴ connectionID : %ld\n", ptr->states.list.array[j]->maneuverAssistList->list.array[q]->connectionID);
                printf("\t   ㄴ pedBicycleDetect : %d\n", *ptr->states.list.array[j]->maneuverAssistList->list.array[q]->pedBicycleDetect);
                sprintf(buf_manueverAssistList, "-mAL[%d] %ld %d",q,ptr->states.list.array[j]->maneuverAssistList->list.array[q]->connectionID,
                *ptr->states.list.array[j]->maneuverAssistList->list.array[q]->pedBicycleDetect);
                strcat(buf_states, buf_manueverAssistList); //maneuverAssistList 데이터 저장
            }
            strcat(buf_intersections, buf_states);
            printf("\n\n");
        }
        strcat(buff, buf_intersections);
        // MISSION : SpaT 메시지에 포함된 Intersection ID 추출
        //           Intersection 내 Ref Position을 기준으로 Offset Node 좌표 추출
        //           Node로 연결된 차선의 Line 별 ID와 SignalGroupID를 출력
    }
    printf("송신 메시지 [%ld bytes] : %s\n", strlen(buff) ,buff);
    udp_sender(buff, "127.0.0.1", 50115);
    udp_sender(buff, "127.0.0.1", 50116);
    return 0;
}


int parse_bsm(BasicSafetyMessage_t *bsm){
    // MISSION : BSM 내 temporary ID 추출
    //           차량의 위치(위도,경도,고도)와 주행 방향, 속도 출력
    return 0;
}