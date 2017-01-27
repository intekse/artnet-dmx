#include "art_net.h"

ArtNetDMX ArtDMX;
extern uint8_t DMXBuffer[DMX_MAX];
static unsigned char buf[BUFFER_SIZE+1];

void artnet_server(void)
{
	uint16_t plen;

	// get the next new packet:
	plen = enc28j60PacketReceive(BUFFER_SIZE, buf);

	// If invalid packet exit and wait for new packet
	if (plen == 0)
	{
		return;
	}

	// Answer arp request and exit function
	if (eth_type_is_arp_and_my_ip(buf, plen))
	{
		make_arp_answer_from_request(buf);
		return;
	}

	// If UDP packet then call appropriate function
	if (eth_type_is_ip_and_my_ip(buf,plen))
	{	
		uint16_t port = get_udp_port();

		if (port == 6454) {
			if(is_art_net(buf, plen, &ArtDMX)) {
				//print_ArtDMX(&ArtDMX);
				for(unsigned int i = 0; i<DMX_MAX; i++) {
					DMXBuffer[i] = ArtDMX.data[i];
				}
			}
			return;
		}

		printf("Port %i is not open\r\n", port);
		return; 
	}

}

uint8_t is_art_net(unsigned char *buf,unsigned int len, ArtNetDMX *ArtDMX)
{
	for(unsigned int i = 0; i<=8; i++) {
		ArtDMX->ID[i] = buf[42 + i];
	}
	ArtDMX->OpCode = buf[51]<<8 | buf[50];
	ArtDMX->version = buf[52]<<8 | buf[53];
	ArtDMX->seq = buf[54];
	ArtDMX->physical = buf[55];
	ArtDMX->Universe = buf[57]<<8 | buf[56];
	ArtDMX->length = buf[58]<<8 | buf[59];
	for(unsigned int i = 0; i<=512; i++) {
		ArtDMX->data[i] = buf[60 + i];
	}

	if(strncmp((char *)ArtDMX->ID, "Art-Net"+0x00, 7)==0 && ArtDMX->OpCode == 0x5000) {
		return 1;
	}

	return 0;
}

uint16_t get_udp_port(void)
{
	uint16_t port;

	port = buf[UDP_DST_PORT_H_P]<<8;
	port |= buf[UDP_DST_PORT_L_P];

	return port;
}

