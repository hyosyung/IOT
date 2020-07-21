#pragma comment(lib,"ws2_32")
#pragma warning(disable: 4996)
#define d(a,b) ((a)-(b))<0?((b)-(a)):((a)-(b))

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <winsock2.h>
#include <conio.h>
#include <time.h>
#include "libemqtt.h"


#define BUF_SIZE   512
#define   SEVER_IP   "192.168.0.93"
#define PORT      1883


//   소켓함수 에러출력 후 종료
void Error_Exit(char* str)
{
	printf("%s Error!! Exit...\n", str);
	_getch();
	exit(1);
}


#define RCVBUFSIZE 1024
uint8_t packet_buffer[RCVBUFSIZE];

int socket_id;
SOCKET hSocket;

int soc_min = 10000;
int soc_max = 0;



int send_packet(void* socket_info, const void* buf, unsigned int count)
{
	SOCKET fd = *((SOCKET*)socket_info);
	return send(fd, buf, count, 0);
}

int init_socket(mqtt_broker_handle_t* broker, const char* hostname, short port)
{
	int flag = 1;
	int keepalive = 300; // Seconds

	WSADATA wsaData;
	SOCKADDR_IN servAdr;

	//   윈속 초기화
	if (WSAStartup(MAKEWORD(2, 2), &wsaData) != 0)
	{
		Error_Exit("WSAStartup()");
	}

	//   소켓 생성: socket()
	hSocket = socket(PF_INET, SOCK_STREAM, 0);   //hSocket는 IAR기준 socket_id를 의미

	if (hSocket > soc_max)soc_max = hSocket;
	if (hSocket < soc_min)soc_min = hSocket;
	printf("soc_max = %d\n", soc_max);
	printf("soc_min = %d\n", soc_min);
	printf("hSocket = %d\n", hSocket);
	if (hSocket == INVALID_SOCKET)
	{
		Error_Exit("socket()");
	}

	//   데이터 초기화 및 connect()
	//printf("try to connect to the server...");
	memset(&servAdr, 0, sizeof(servAdr));
	servAdr.sin_family = AF_INET;
	servAdr.sin_addr.s_addr = inet_addr(hostname);
	servAdr.sin_port = htons(port);

	if (connect(hSocket, (SOCKADDR*)&servAdr, sizeof(servAdr)) == SOCKET_ERROR)
	{
		Error_Exit("connect()");
	}
	//printf("\nserver connected...\n");
	//printf("[send]: ");

	// MQTT stuffs
	mqtt_set_alive(broker, keepalive);
	broker->socket_info = (void*)&hSocket;
	broker->send = send_packet;

	return 0;
}

int close_socket(mqtt_broker_handle_t* broker)
{
	SOCKET fd = *((SOCKET*)broker->socket_info);
	closesocket(fd);
	return WSACleanup();
}


int read_packet(int timeout)
{
	int total_bytes = 0, bytes_rcvd, packet_length;

	memset(packet_buffer, 0, sizeof(packet_buffer));

	while (total_bytes < 2) // Reading fixed header
	{
		if ((bytes_rcvd = recv(hSocket, (packet_buffer + total_bytes), RCVBUFSIZE, 0)) <= 0)
			return -1;
		total_bytes += bytes_rcvd; // Keep tally of total bytes

	}

	packet_length = packet_buffer[1] + 2; // Remaining length + fixed header length
	while (total_bytes < packet_length) // Reading the packet
	{
		if ((bytes_rcvd = recv(hSocket, (packet_buffer + total_bytes), RCVBUFSIZE, 0)) <= 0)
			return -1;
		total_bytes += bytes_rcvd; // Keep tally of total bytes
	}
	return packet_length;

}

int main(void)
{
	int a = 75, b = 75, c = 75, avr;

	int packet_length;
	uint16_t msg_id,msg_id_2, msg_id_rcv,msg_id_rcv_2;
	mqtt_broker_handle_t broker;




	mqtt_init(&broker, "PC_SIGNAL");
	mqtt_init_auth(&broker, NULL, NULL);


	init_socket(&broker, SEVER_IP, PORT);

	mqtt_connect(&broker);


	mqtt_subscribe(&broker, "B_SIGNAL", &msg_id);
	mqtt_subscribe(&broker, "ERROR", &msg_id_2);
	//while (1) {
	// >>>>> CONNECT







	// <<<<< CONNACK
	packet_length = read_packet(0);

	printf("receive packet length = %d\n", packet_length);

	if (packet_length < 0)
	{
		fprintf(stderr, "Error(%d) on read packet!\n", packet_length);
		//return -1;
	}

	if (MQTTParseMessageType(packet_buffer) != MQTT_MSG_CONNACK)
	{
		fprintf(stderr, "CONNACK expected!\n");
		//return -2;
	}

	if (packet_buffer[3] != 0x00)
	{
		fprintf(stderr, "CONNACK failed!\n");
		//return -2;
	}

	printf("CONNACK received..\n");


	int Time_First = clock();
	// >>>>> PUBLISH QoS 0
	printf("subscribe\n");


	// <<<<< SUBACK
	packet_length = read_packet(1);
	if (packet_length < 0)
	{
		fprintf(stderr, "Error(%d) on read packet!\n", packet_length);
		//return -1;
	}

	if (MQTTParseMessageType(packet_buffer) != MQTT_MSG_SUBACK)
	{
		fprintf(stderr, "SUBACK expected!\n");
		//return -2;
	}

	msg_id_rcv = mqtt_parse_msg_id(packet_buffer);
	if (msg_id != msg_id_rcv)
	{
		fprintf(stderr, "%d message id was expected, but %d message id was found!\n", msg_id, msg_id_rcv);
		//return -3;
	}
	int flag = 1;

	while (1)
	{
		msg_id_rcv = mqtt_parse_msg_id(packet_buffer);

		if (flag == 1) {
			packet_length = read_packet(0);
			if (packet_length == -1)
			{
				fprintf(stderr, "Error(%d) on read packet!\n", packet_length);
				//return -1;
			}
			else if (packet_length > 0)
			{
				printf("Packet Header: 0x%x...\n", packet_buffer[0]);
				if (MQTTParseMessageType(packet_buffer) == MQTT_MSG_PUBLISH)
				{
					uint8_t topic[255], msg[1000];
					uint16_t len;
					len = mqtt_parse_pub_topic(packet_buffer, topic);
					topic[len] = '\0'; // for printf
					len = mqtt_parse_publish_msg(packet_buffer, msg);
					msg[len] = '\0'; // for printf

					

					if (!strcmp(topic, "ERROR")) {
						if (msg[0] == '0') {
							printf("고장난 보드 없음!");
							//mqtt_publish(&broker, "S_SIGNAL", "0", 1);
						}
						else if (msg[0] == '1') {
							printf("1번 고장\n");
							time_t ltime;
							struct tm *today;
							time(&ltime);
							today = localtime(&ltime);

							printf("고장 시각 : %04d-%02d-%02d %02d:%02d:%02d\n",
								today->tm_year + 1900,   // tm_year는 1900을 더해야 서기 연도가 됨
								today->tm_mon + 1, // tm_mon은 1월이 0, 2월이 1... 식으로 되어 있음 
								today->tm_mday,
								today->tm_hour,
								today->tm_min,
								today->tm_sec);
							while (1) {
								char ch = 0;
								ch = getch();
								if (ch == 27)
									break;
							}

						}
						else if (msg[0] == '2') {
							printf("2번 고장\n");
							time_t ltime;
							struct tm *today;
							time(&ltime);
							today = localtime(&ltime);

							printf("고장 시각 : %04d-%02d-%02d %02d:%02d:%02d\n",
								today->tm_year + 1900,   // tm_year는 1900을 더해야 서기 연도가 됨
								today->tm_mon + 1, // tm_mon은 1월이 0, 2월이 1... 식으로 되어 있음 
								today->tm_mday,
								today->tm_hour,
								today->tm_min,
								today->tm_sec);
							while (1) {
								char ch = 0;
								ch = getch();
								if (ch == 27)
									break;
							}
						}
						else if (msg[0] == '3') {
							printf("3번 고장\n");
							time_t ltime;
							struct tm *today;
							time(&ltime);
							today = localtime(&ltime);

							printf("고장 시각 : %04d-%02d-%02d %02d:%02d:%02d\n",
								today->tm_year + 1900,   // tm_year는 1900을 더해야 서기 연도가 됨
								today->tm_mon + 1, // tm_mon은 1월이 0, 2월이 1... 식으로 되어 있음 
								today->tm_mday,
								today->tm_hour,
								today->tm_min,
								today->tm_sec);

							while (1) {
								char ch = 0;
								ch = getch();
								if (ch == 27)
									break;
							}
						}
					}

					else if(!strcmp(topic,"B_SIGNAL")) {
						printf("토픽: %s, 센서값: %d%d, 보드번호: %d\n", topic, msg[0] - '0', msg[1] - '0', msg[3] - '0');
						switch (msg[3] - '0') {
						case 1:
							a = (int)(msg[0] - '0') * 10 + (int)(msg[1] - '0'); break;
						case 2:
							b = (int)(msg[0] - '0') * 10 + (int)(msg[1] - '0'); break;
						case 3:
							c = (int)(msg[0] - '0') * 10 + (int)(msg[1] - '0'); break;
						}

						avr = (a + b + c) / 3;
						printf("average : %d \n", avr);
						printf("a: %d, b: %d, c: %d\n\n", a, b, c);

						int d, e, f;
						d = d(avr, a); e = d(avr, b); f = d(avr, c);
						if (d > 20 && a < b&&a < c&&b>avr&&c>avr) {
							mqtt_publish(&broker, "S_SIGNAL", "1", 0);
							printf("1번 보드 청소\n");
						}
						else if (e > 20 && b < a&&b < c&&a>avr&&c>avr) {
							mqtt_publish(&broker, "S_SIGNAL", "2", 0);
							printf("2번 보드 청소\n");
						}
						else if (f > 20 && c < a&&c < b&&a>avr&&b>avr) {
							mqtt_publish(&broker, "S_SIGNAL", "3", 0);
							printf("3번 보드 청소\n");
						}
						else {
							mqtt_publish(&broker, "S_SIGNAL", "0", 2);
						}
					}
				}
			}



		}
		

		
	}
	return 0;
}