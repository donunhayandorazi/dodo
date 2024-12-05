#ifndef MCP3008_H
#define MCP3008_H

#include <wiringPi.h>
#include <wiringPiSPI.h>

#define SPI_CHANNEL 0   // SPI 채널 0 (CE0)
#define SPI_SPEED 1000000 // SPI 속도 (1 MHz)

void mcp3008Setup(int channel, int spi_channel);
int mcp3008Read(int channel);

#endif
