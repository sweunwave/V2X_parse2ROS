#include <iostream>

#include <thread>
#include <arpa/inet.h>
#include <netinet/tcp.h>
#include <string.h>

#include "obu_packet.h"

#define MAX_BUF_LEN  4096
int clnt_sock = -1;
// std::string ip = "192.168.10.10";
std::string ip = "192.168.10.10";

uint32_t port = 24000;

int Connect_socket();
int Disconnect_socket();

void PrintHeader(ObuTcpHeader *msg);
void PrintMissionList(MissionList *msg);
void PrintMissionStatus(MissionStatus *msg);
void PrintRequest(AutonomousRequest *msg);
void PrintRequest_ack(AutonomousRequest_ACK *msg);

int main()
{ 
    int sock_connect_state = -1;
    uint8_t seq = 0;
    while (true)
    {
        if(sock_connect_state == -1)
        {
            sock_connect_state = Connect_socket();
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));//reconnect period
            continue;
        }

        unsigned char recvbuf[MAX_BUF_LEN] = {0,};
        int len = recv(clnt_sock, recvbuf, sizeof(recvbuf), 0); //blocking function!
        if(len <=0)
        {
            sock_connect_state = Disconnect_socket();
            continue;
        }

        if(len < sizeof(ObuTcpHeader))
        {
            printf("recv unkown short packet! (%d byte)\n", len);
            continue;
        }
        ObuTcpHeader *header = (ObuTcpHeader*)recvbuf;
       
        if(len - sizeof(ObuTcpHeader) < header->payload_length )
        {
            printf("pay size error : %d(recv) < %d(value)\n", len - sizeof(ObuTcpHeader), header->payload_length);
            continue;
        }
        
        switch (header->message_type)
        {
        case MessageType::MISSION_LIST :
            {              
                MissionList msg ={0,};
                size_t copy_len_1 = sizeof(MissionList) - sizeof(MissionList::mission_list) - sizeof(MissionList::irregular_list);
                memcpy(&msg,recvbuf,copy_len_1);

                size_t copy_len_2 = sizeof(MissionData) * msg.total_mission_count;
                msg.mission_list = new MissionData[msg.total_mission_count];
                memcpy(msg.mission_list, (void*)&recvbuf[copy_len_1],copy_len_2);

                size_t copy_len_3 = sizeof(IrregularData) * msg.irregular_count;
                msg.irregular_list = new IrregularData[msg.irregular_count];
                memcpy(msg.irregular_list, (void*)&recvbuf[copy_len_1 + copy_len_2], copy_len_3);

                PrintMissionList(&msg);

                //send mission request msg
                AutonomousRequest req = {0,};
                req.header.message_type = MessageType::AUTONOMOUS_REQUEST;
                req.header.sequence = seq++;
                req.header.payload_length = sizeof(AutonomousRequest) - sizeof(ObuTcpHeader);
                req.header.device_type = 0xCE;
                req.header.device_id[0] = 0x04;
                req.header.device_id[1] = 0x05;
                req.header.device_id[2] = 0x06;
                
                req.mission_id = 17; 
                // req.request = RequestCode::NOT_REQ;
                req.request = 0x01;
                req.response = 0x00; //0
                sprintf(req.description,"BISA");
                req.temporary; //all 0
                req.end_point = 0x0D0A;
                
                PrintRequest(&req);

                int len = send(clnt_sock, (char*)&req, sizeof(req), 0); 
                
                delete msg.mission_list;
                delete msg.irregular_list;
            }
            
            break;
        case MessageType::MISSION_STATUS :
            {
                MissionStatus *msg = (MissionStatus*)recvbuf;
                PrintMissionStatus(msg);
            }
            break;
        case MessageType::AUTONOMOUS_REQUEST_ACK :
            {
                AutonomousRequest_ACK *msg = (AutonomousRequest_ACK*)recvbuf;
                PrintRequest_ack(msg);
                
                if(msg->request == RequestCode::MISSION_REQ )
                {
                    if(msg->response == ResponseCode::SEND_SERVER)
                    {
                        printf("[AUTONOMOUS_REQUEST_ACK] MISSION_REQ : SEND_SERVER\n ");
                    }
                    else if(msg->response == ResponseCode::MISSION_CONFIRM)
                    {
                        printf("[AUTONOMOUS_REQUEST_ACK] MISSION_REQ : MISSION_CONFIRM\n");

                        //send START_POINT_REQ
                        AutonomousRequest req = {0,};
                        req.header.message_type = MessageType::AUTONOMOUS_REQUEST;
                        req.header.sequence = seq++;
                        req.header.payload_length = sizeof(AutonomousRequest) - sizeof(ObuTcpHeader);
                        req.header.device_type = 0xCE;
                        req.header.device_id[0] = 0x04;
                        req.header.device_id[1] = 0x05;
                        req.header.device_id[2] = 0x06;

                        req.mission_id = 17;
                        req.request = 0x02;
                        // req.request = RequestCode::NOT_REQ;
                        req.response = 0x00; //0
                        sprintf(req.description,"BISA");
                        req.temporary; //all 0
                        req.end_point = 0x0D0A;
                        
                        PrintRequest(&req);
                
                        int lens = send(clnt_sock, (char*)&req, sizeof(req), 0); 
                    }
                    else
                    {   
                        printf("[AUTONOMOUS_REQUEST_ACK] MISSION_REQ : ERROR(%02X)\n", msg->response);
                    }
                }

                if(msg->request == RequestCode::START_POINT_REQ)
                {
                    if(msg->response == ResponseCode::SEND_SERVER)
                    {
                        printf("[AUTONOMOUS_REQUEST_ACK] START_POINT_REQ : SEND_SERVER\n ");
                    }
                    else if(msg->response == ResponseCode::START_POINT_CONFIRM)
                    {
                        printf("[AUTONOMOUS_REQUEST_ACK] START_POINT_REQ : START_POINT_CONFIRM\n");

                        //send END_POINT_REQ
                        AutonomousRequest req = {0,};
                        req.header.message_type = MessageType::AUTONOMOUS_REQUEST;
                        req.header.sequence = seq++;
                        req.header.payload_length = sizeof(AutonomousRequest) - sizeof(ObuTcpHeader);
                        req.header.device_type = 0xCE;
                        req.header.device_id[0] = 0x04;
                        req.header.device_id[1] = 0x05;
                        req.header.device_id[2] = 0x06;

                        req.mission_id = 17;
                        req.request = 0x03;
                        // req.request = RequestCode::NOT_REQ;
                        req.response = 0x00; //0
                        sprintf(req.description,"BISA");
                        req.temporary; //all 0
                        req.end_point = 0x0D0A;
                        
                        PrintRequest(&req);
                        
                        int lens = send(clnt_sock, (char*)&req, sizeof(req), 0); 
                    }
                    else
                    {   
                        printf("[AUTONOMOUS_REQUEST_ACK] START_POINT_REQ : ERROR(%02X)\n", msg->response);
                    }
                }

                if(msg->request == RequestCode::END_POINT_REQ)
                {
                     if(msg->response == ResponseCode::SEND_SERVER)
                    {
                        printf("[AUTONOMOUS_REQUEST_ACK] END_POINT_REQ : SEND_SERVER\n ");
                    }
                    else if(msg->response == ResponseCode::END_POINT_CONFIRM)
                    {
                        printf("[AUTONOMOUS_REQUEST_ACK] END_POINT_REQ : END_POINT_CONFIRM\n");
                        printf("\n");
                        printf("===================================================\n");
                        printf("================ Mission complete! ================\n");
                        printf("===================================================\n");
                    }
                    else
                    {   
                        printf("[AUTONOMOUS_REQUEST_ACK] END_POINT_REQ : ERROR(%02X)\n", msg->response);
                    }
                }
            }
            break;
        default:
            printf("recv unkown message\n");
            break;
        }
    }
    
    Disconnect_socket();

    return 0;
}

int Connect_socket()
{
    if(clnt_sock == -1)
    {
        printf("aleady create sock!!\n");
    }
    clnt_sock = socket(PF_INET,SOCK_STREAM,0);
    if(clnt_sock == -1)
    {
        printf("sock create error\n");
        return -1;
    }

    int opt_val = 1;
    setsockopt(clnt_sock, IPPROTO_TCP, TCP_NODELAY, (void*)&opt_val, sizeof(opt_val));
    setsockopt(clnt_sock, SOL_SOCKET, SO_REUSEADDR, (void*)&opt_val, sizeof(opt_val));
    
    sockaddr_in serv_addr;
    memset(&serv_addr,0,sizeof(serv_addr));
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_addr. s_addr= inet_addr(ip.c_str());
    serv_addr.sin_port = htons(port);
   
    int res = -1;
    res = connect(clnt_sock,(sockaddr*)&serv_addr,sizeof(serv_addr));
    if(res == -1)
    {
        printf("sock connect error\n");
    }
    else
    {
        printf("sock connect success\n");
    }
    return res;
}

int Disconnect_socket()
{
    if(clnt_sock != -1)
    {
        shutdown(clnt_sock, SHUT_RDWR);
        close(clnt_sock);
        clnt_sock=-1;
        printf("sock disconnect\n");
    }    
    return -1;
}

void PrintHeader(ObuTcpHeader *msg)
{
    if(msg == nullptr)
    {
        return;
    }
    printf("\n");
    printf("[Print Message Header]\n");
    printf("\theader.message_type : 0x%04X\n", msg->message_type);
    printf("\theader.sequence : %d\n", msg->sequence);
    printf("\theader.payload_length : %d\n", msg->payload_length);
    printf("\theader.device_type : 0x%02X\n", msg->device_type);
    printf("\theader.device_id : 0x%02X 0x%02X 0x%02x\n", msg->device_id[0], msg->device_id[1], msg->device_id[2]);
}

void PrintMissionList(MissionList *msg)
{
    if(msg == nullptr)
    {
        return;
    }
    if(msg->header.message_type != MessageType::MISSION_LIST)
    {
        return;
    }
    
    printf("\n");
    printf("[Print Mission List]\n");
    printf("\theader.message_type : 0x%04X\n", msg->header.message_type);
    printf("\theader.sequence : %d\n", msg->header.sequence);
    printf("\theader.payload_length : %d\n", msg->header.payload_length);
    printf("\theader.device_type : 0x%02X\n", msg->header.device_type);
    printf("\theader.device_id : 0x%02X 0x%02X 0x%02x\n", msg->header.device_id[0], msg->header.device_id[1], msg->header.device_id[2]);
    
    printf("\tpay.total_mission_count : %d\n", msg->total_mission_count);
    printf("\tpay.available_count : %d\n", msg->available_count);
    printf("\tpay.irregular_count : %d\n", msg->irregular_count);

    for(int i=0; i<msg->total_mission_count; i++)
    {
        printf("\tpay.mission_list[%d].mission_id : %d\n", i, msg->mission_list[i].mission_id);
        printf("\tpay.mission_list[%d].status : %d\n", i, msg->mission_list[i].status);
        printf("\tpay.mission_list[%d].included_mission : 0x%02X\n", i, msg->mission_list[i].included_mission);
        printf("\tpay.mission_list[%d].score : %d\n", i, msg->mission_list[i].score);
        printf("\tpay.mission_list[%d].distance : %d\n", i, msg->mission_list[i].distance);
        printf("\tpay.mission_list[%d].irregular_id : %d\n", i, msg->mission_list[i].irregular_id);
        printf("\tpay.mission_list[%d].start_lat : %lf\n", i, msg->mission_list[i].start_lat);
        printf("\tpay.mission_list[%d].start_Lon : %lf\n", i, msg->mission_list[i].start_Lon);
        printf("\tpay.mission_list[%d].end_lat : %lf\n", i, msg->mission_list[i].end_lat);
        printf("\tpay.mission_list[%d].end_lon : %lf\n", i, msg->mission_list[i].end_lon);
    }

    for(int i=0; i<msg->irregular_count; i++)
    {
        printf("\tpay.irregular_list[%d].irregular_id : %d\n", i, msg->irregular_list[i].irregular_id);
        printf("\tpay.irregular_list[%d].lat_1 : %lf\n", i, msg->irregular_list[i].lat_1);
        printf("\tpay.irregular_list[%d].lon_1 : %lf\n", i, msg->irregular_list[i].lon_1);
        printf("\tpay.irregular_list[%d].lat_2 : %lf\n", i, msg->irregular_list[i].lat_2);
        printf("\tpay.irregular_list[%d].lon_2 : %lf\n", i, msg->irregular_list[i].lon_2);
        printf("\tpay.irregular_list[%d].lat_3 : %lf\n", i, msg->irregular_list[i].lat_3);
        printf("\tpay.irregular_list[%d].lon_3 : %lf\n", i, msg->irregular_list[i].lon_3);
        printf("\tpay.irregular_list[%d].lat_4 : %lf\n", i, msg->irregular_list[i].lat_4);
        printf("\tpay.irregular_list[%d].lon_4 : %lf\n", i, msg->irregular_list[i].lon_4);
    }
}
void PrintMissionStatus(MissionStatus *msg)
{
    if(msg == nullptr)
    {
        return;
    }
    if(msg->header.message_type != MessageType::MISSION_STATUS)
    {
        return;
    }
    printf("\n");
    printf("[Print Mission Status]\n");
    printf("\theader.message_type : 0x%04X\n", msg->header.message_type);
    printf("\theader.sequence : %d\n", msg->header.sequence);
    printf("\theader.payload_length : %d\n", msg->header.payload_length);
    printf("\theader.device_type : 0x%02X\n", msg->header.device_type);
    printf("\theader.device_id : 0x%02X 0x%02X 0x%02x\n", msg->header.device_id[0], msg->header.device_id[1], msg->header.device_id[2]);

    printf("\tpay.v2x_server_connected : %d\n", msg->v2x_server_connected);
    printf("\tpay.mission_status : %d\n", msg->mission_status);
    printf("\tpay.progressed_mission_id : %d\n", msg->progressed_mission_id);
    printf("\tpay.progressed_mission_status : %d\n", msg->progressed_mission_status);
}
void PrintRequest(AutonomousRequest *msg)
{
    if(msg == nullptr)
    {
        return;
    }
    if(msg->header.message_type != MessageType::AUTONOMOUS_REQUEST)
    {
        return;
    }

    printf("\n");
    printf("[Print Request]\n");
    printf("\theader.message_type : 0x%04X\n", msg->header.message_type);
    printf("\theader.sequence : %d\n", msg->header.sequence);
    printf("\theader.payload_length : %d\n", msg->header.payload_length);
    printf("\theader.device_type : 0x%02X\n", msg->header.device_type);
    printf("\theader.device_id : 0x%02X 0x%02X 0x%02x\n", msg->header.device_id[0], msg->header.device_id[1], msg->header.device_id[2]);

    printf("\tpay.mission_id : %d\n",msg->mission_id);
    printf("\tpay.request : %d\n",msg->request);
    printf("\tpay.response : %d\n",msg->response);
    printf("\tpay.description : %s\n",msg->description);
    printf("\tpay.temporary : %s\n",msg->temporary);
    printf("\tpay.end_point : 0x%04X\n",msg->end_point);
}
void PrintRequest_ack(AutonomousRequest_ACK *msg)
{
    if(msg == nullptr)
    {
        return;
    }
    if(msg->header.message_type != MessageType::AUTONOMOUS_REQUEST_ACK)
    {
        return;
    }

    printf("\n");
    printf("[Print Request Ack]\n");
    printf("\theader.message_type : 0x%04X\n", msg->header.message_type);
    printf("\theader.sequence : %d\n", msg->header.sequence);
    printf("\theader.payload_length : %d\n", msg->header.payload_length);
    printf("\theader.device_type : 0x%02X\n", msg->header.device_type);
    printf("\theader.device_id : 0x%02X 0x%02X 0x%02x\n", msg->header.device_id[0], msg->header.device_id[1], msg->header.device_id[2]);

    printf("\tpay.mission_id : %d\n",msg->mission_id);
    printf("\tpay.request : %d\n",msg->request);
    printf("\tpay.response : %d\n",msg->response);
    printf("\tpay.description : %s\n",msg->description);
    printf("\tpay.temporary : %s\n",msg->temporary);
    printf("\tpay.end_point : 0x%04X\n",msg->end_point);
}