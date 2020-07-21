#include "stm32f10x.h"
#include "st7735.h"
#include "udf.h"
#include "option.h"
#include <stdio.h>
#include <string.h>
#include "libemqtt.h"

#define printf		Uart1_Printf
#define BUF_SIZE	512
#define	SEVER_IP	"192.168.0.93"
#define PORT		1883

#define PANEL_NUM       1+'0'
#define EFFICIENCY_10   0
#define EFFICIENCY_1    0

#define BASE  (500) // MUSIC msec

#define RCVBUFSIZE 1024
uint8_t packet_buffer[RCVBUFSIZE];


int socket_id;
mqtt_broker_handle_t broker;
int keepalive = 0;

//music
enum key{C1, C1_, D1, D1_, E1, F1, F1_, G1, G1_, A1, A1_, B1, C2, C2_, D2, D2_, E2, F2, F2_, G2, G2_, A2, A2_, B2};
enum note{N16=BASE/4, N8=BASE/2, N4=BASE, N2=BASE*2, N1=BASE*4};
const int song1[][2] = {{C1, N2}, {E2, N2}};
const char * note_name[] = {"C1", "C1#", "D1", "D1#", "E1", "F1", "F1#", "G1", "G1#", "A1", "A1#", "B1", "C2", "C2#", "D2", "D2#", "E2", "F2", "F2#", "G2", "G2#", "A2", "A2#", "B2"};


extern volatile int ADC1_value;
extern volatile int ADC1_flag;
extern volatile int TIM4_Expired;

extern mqtt_broker_handle_t broker;
extern CHCONFIG_TYPE_DEF Chconfig_Type_Def;

RCC_ClocksTypeDef RCC_ClockFreq;

/* Private function prototypes -----------------------------------------------*/
extern void WizFi250_Serial_Input_Mode(void);
extern volatile int wifi_mode;

extern void Ethernet_Test(void);
extern void Ethernet_Init(void);

extern void loopback_tcps(char s, unsigned short port);
extern void loopback_tcpc(char s, char *str_ip,unsigned short port);
extern void loopback_udp(char s, unsigned short port);

extern void LED_Init(void);
extern void LED_Display(unsigned int num);

extern void Key_ISR_Enable(int en);
extern void Key_Poll_Init(void);
extern int Key_Get_Pressed(void);

extern void ADC_Configuration(void);
extern void TIM_Delay_ms(unsigned int time);

extern void TIM3_Buzzer_Beep(int tone, int duration);
extern void TIM3_Buzzer_Init(void);
extern void TIM2_Delay(int time);

extern void LED_Init(void);
extern void LED_Display(unsigned int num);
extern void LED_All_On(void);
extern void LED_All_Off(void);

// wifi_ex.c
//extern void Disp_Help(void);
//extern void Factory_Default(void);
//extern void WiFi_Init(void);
//extern void SPI2_Receive(void);
//extern unsigned char SPI2_SendByte(unsigned char byte);
//extern void SPI2_Send_String(unsigned char *pt);
//extern int Check_Cmd(int n);
//extern void WizFi250_Serial_Data_Mode(void);
//extern int	Run_Server(void);


/* Graphic functions ---------------------------------------------------------*/
void Graphic_eff(char* value)
{
    Lcd_Clr_Screen(BLACK);
    Lcd_Draw_Line(10,50,50,10,YELLOW);
    Lcd_Draw_Hline(50,10,50,YELLOW);
    Lcd_Draw_Line(50,50,10,110,YELLOW);
    Lcd_Printf(65,0,GREEN,BLACK,5,5,value);
}

void Graphic_clean(void)
{
  int i;
  
      Lcd_Clr_Screen(BLACK);
      
      Lcd_Printf(15,70,BLUE2,BLACK,2,2,"CLEANING");
      Lcd_Draw_Rect(8,20,150,40,BLUE2);
      for(i=8; i<=150; i+=3)
      {
        Lcd_Draw_Bar(8,20,i,40,BLUE);
        TIM_Delay_ms(50);
      }
}

void Graphic_error(void)
{
    Lcd_Printf(40,50,RED,BLACK,2,2,"ERROR");

}


/* Private functions ---------------------------------------------------------*/
void Board_Init(void);

int send_packet(void* socket_info, const void* buf, unsigned int count)
{
	int8_t fd = *((int8_t*)socket_info);
	return send(fd, (unsigned char*)buf, count);
}

int init_socket(mqtt_broker_handle_t* broker, const char* hostname, short port)
{
	socket_id = PANEL_NUM - '0';
	socket_id = socket(socket_id, SOCK_STREAM, 8522, 0x00 );
	if( socket_id  < 0)
	{
		printf("socket create error\n");
		return -1;
	}
        
        printf("socket_id : %d\n",socket_id);

	if( ( connect(socket_id,Chconfig_Type_Def.destip, port ) ) < 0)
		return -1;

	// MQTT stuffs
	mqtt_set_alive(broker, keepalive);
	broker->socket_info = (void*)&socket_id;
	broker->send = send_packet;

	return 0;
}

int close_socket(mqtt_broker_handle_t* broker)
{
	int8_t fd = *((int8_t*)broker->socket_info);
	return close(fd);
}


int read_packet(int timeout)
{
	int total_bytes = 0, bytes_rcvd, packet_length;

	memset(packet_buffer, 0, sizeof(packet_buffer));

	while(total_bytes < 2) // Reading fixed header
	{
		if((bytes_rcvd = recv(socket_id, (packet_buffer+total_bytes), RCVBUFSIZE)) <= 0)
			return -1;
		total_bytes += bytes_rcvd; // Keep tally of total bytes

	}

	packet_length = packet_buffer[1] + 2; // Remaining length + fixed header length
	while(total_bytes < packet_length) // Reading the packet
	{
		if((bytes_rcvd = recv(socket_id, (packet_buffer+total_bytes), RCVBUFSIZE)) <= 0)
			return -1;
		total_bytes += bytes_rcvd; // Keep tally of total bytes
	}
	return packet_length;

}
void main(void)
{
        char SIGNAL[10] = {EFFICIENCY_10, EFFICIENCY_1, ' ', PANEL_NUM};
	int packet_length;
        int A;
        int k;
        int i;
        int CLEAN_cnt = 0;
	uint16_t msg_id, msg_id_rcv;
        

	RCC_GetClocksFreq(&RCC_ClockFreq);
        
        // Setting initialize
        Uart1_Init(115200);
        printf("Uart1_Init SUCCESS\n");        
        
        Uart2_Init(115200);
        printf("Uart2_Init SUCCESS\n");
        
	TIM_Config();
        printf("TIM_Config SUCCESS\n");
        
        Lcd_Init();
        printf("Lcd_Init SUCCESS\n");
        
        LED_Init();
        printf("LED_Init SUCCESS\n");	
        
        Ethernet_Init();
        printf("Ethernet_Init SUCCESS\n");
        
//        WiFi_Init();
//        printf("WiFi_Init SUCCESS\n");
        
//        Factory_Default();
//        printf("Factory_Default SUCCESS\n");
          
        // >>>>> CONNECT
        mqtt_init(&broker, "hyosung_1");
        printf("\nmqtt_init SUCCESS\n");
        
        mqtt_init_auth(&broker, NULL, NULL);
        printf("mqtt_init_auth SUCCESS\n");
        
        init_socket(&broker, SEVER_IP, PORT);
        printf("init_socket SUCCESS\n");
        
        mqtt_connect(&broker);
        printf("mqtt_connect SUCCESS\n");
        
        TIM2_Delay(1000);
        
        

        // <<<<< CONNACK
        packet_length = read_packet(0);
        

        printf("receive packet length = %d\n",packet_length);

        if(packet_length < 0)
        {
                printf("Error(%d) on read packet!\n", packet_length);
                //return;
        }

        if(MQTTParseMessageType(packet_buffer) != MQTT_MSG_CONNACK)
        {
                printf("CONNACK expected!\n");
                //return;
        }

        if(packet_buffer[3] != 0x00)
        {
                printf("CONNACK failed!\n");
                //return;
        }

        printf("CONNACK received..\n");


        // >>>>>> subscribe        
        mqtt_subscribe(&broker, "S_SIGNAL", &msg_id);
        TIM2_Delay(100);
        

        
        TIM_Delay_ms(100);

        // <<<<< SUBACK
        packet_length = read_packet(1);
        if(packet_length < 0)
        {
                printf("Error(%d) on read packet!\n", packet_length);
                //return ;
        }

        if(MQTTParseMessageType(packet_buffer) != MQTT_MSG_SUBACK)
        {
                printf("SUBACK expected!\n");
                //return ;
        }

        msg_id_rcv = mqtt_parse_msg_id(packet_buffer);
        if(msg_id != msg_id_rcv)
        {
                printf("%d message id was expected, but %d message id was found!\n", msg_id, msg_id_rcv);
                //return ;
        }
        
        printf("SUBACK received..\n");
        
        
        mqtt_publish(&broker, "ERROR", "0", 2);

        
        while(1)
        {
                //ADC Check & Check Efficiency
                ADC_Configuration();
                A=(ADC1_value*100)/4096;
                Uart1_Printf("The current AD value = %d% \r\n", A);
                SIGNAL[0]=A/10+'0';
                SIGNAL[1]=A%10+'0';
                mqtt_publish(&broker, "B_SIGNAL", SIGNAL, 1);
                
                // Display Efficiency
                Graphic_eff(SIGNAL);
                
                // Print Subscribe Message
                packet_length = read_packet(0);
                if(packet_length == -1)
                {
                        printf("Error(%d) on read packet!\n", packet_length);
                        //return ;
                }
                else if(packet_length > 0)
                {
                        printf("Packet Header: 0x%x...\n", packet_buffer[0]);
                        uint8_t topic[255], msg[1000];
                        uint16_t len;
                        len = mqtt_parse_pub_topic(packet_buffer, topic);
                        topic[len] = '\0'; // for printf
                        len = mqtt_parse_publish_msg(packet_buffer, msg);
                        msg[len] = '\0'; // for printf
                        printf("%s %s\n", topic, msg);
                                
                        
                        
                        // Configure Message and Check PANEL Number & Contorl
                        if(msg[0] == PANEL_NUM)
                        {
                                  Lcd_Clr_Screen(BLACK);
                        
                                  Lcd_Printf(15,70,BLUE2,BLACK,2,2,"CLEANING");
                                  Lcd_Draw_Rect(8,20,150,40,BLUE2);
                                  for(i=8; i<=150; i+=3)
                                  {
                                            ADC_Configuration();
                                            A=(ADC1_value*100)/4096;
                                            SIGNAL[0]=A/10+'0';
                                            SIGNAL[1]=A%10+'0';
                                            mqtt_publish(&broker, "B_SIGNAL", SIGNAL, 1);     
                                            Lcd_Draw_Bar(8,20,i,40,BLUE);
                                            packet_length = read_packet(0);
                                            
                                  }
                        
                                 CLEAN_cnt++;
                        }
                        
                        
                        // FAULTS CLEAR
                        if(msg[0] == '0')
                        {
                                 CLEAN_cnt = 0;
                        }
                        
                        

                }
                
                // PANEL FAULTS
                if(CLEAN_cnt >= 5)      
                {
                    // Display ERROR
                    Graphic_error();    
                    

                    mqtt_publish(&broker, "ERROR", "1", 2);//추가됨 에러가 났을때 publish, 1번보드가 error이므로 1

                    

                    while(1)
                    {   
                        int x=0;
                        TIM3_Buzzer_Init();
                        for(k=0; k<(sizeof(song1)/sizeof(song1[0])); k++)
                        {
                          TIM3_Buzzer_Beep(song1[k][0], song1[k][1]);
                          LED_Display(x=x^1);
                        }
                    }
                }
                
       }
}
 

