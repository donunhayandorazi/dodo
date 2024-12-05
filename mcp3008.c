#include "mcp3008.h"

void mcp3008Setup(int channel, int spi_channel) {
    // WiringPi SPI 초기화
    if (wiringPiSPISetup(spi_channel, SPI_SPEED) == -1) {
        printf("SPI Setup Failed!\n");
        exit(1);
    }
}

int mcp3008Read(int channel) {
    unsigned char buffer[3];
    unsigned char rx_buffer[3];
    int result;

    if (channel < 0 || channel > 7) {
        printf("Invalid channel: %d\n", channel);
        return -1;
    }

    buffer[0] = 1;  // Start bit
    buffer[1] = (8 + channel) << 4;  // Channel 설정
    buffer[2] = 0;  // Don't care, 0으로 설정

    // SPI 데이터 전송
    wiringPiSPIDataRW(SPI_CHANNEL, buffer, 3);

    // 10비트 ADC 값 추출
    result = ((rx_buffer[1] & 3) << 8) | rx_buffer[2];
    
    return result;
}
