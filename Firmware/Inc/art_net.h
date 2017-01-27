
#ifndef ART_NET_H
#define ART_NET_H

#include "stm32f1xx_hal.h"
#include <string.h>

#include "net.h"
#include "def.h"
#include "enc28j60.h"
#include "ip_arp_udp_tcp.h"

typedef struct {
	char ID[8]; 		// "Art-Net"
	uint16_t OpCode;
	uint16_t version;
	uint8_t seq;
	uint8_t physical; 	// 0x00
	uint16_t Universe;		// low universe (0-255)
	uint8_t net;		// high universe (not used)
	uint16_t length;	// data length (2-512)
	uint8_t data[512];	// uiniverse data
} ArtNetDMX;

void artnet_server(void);
uint8_t is_art_net(unsigned char *buf,unsigned int len, ArtNetDMX *ArtDMX);
uint16_t get_udp_port(void);

#endif
