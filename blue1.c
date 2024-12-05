#include <stdio.h>
#include <stdlib.h>
#include <wiringPi.h>
#include <wiringPiI2C.h>
#include <unistd.h>
#include <time.h>
#include <mcp3008.h> // ADC 아날로그 변환기 // * 교수님께 달라고 하기, 배터리 담을 거 
#include <Adafruit_SSD1306.h>
#include <Adafruit_GFX.h>
#include <bluetooth/bluetooth.h>
#include <bluetooth/rfcomm.h>
#include <bluetooth/sco.h>

#define SERIAL_PORT "/dev/ttyAMA0"  // 블루투스 모듈이 연결된 포트

//배터리 전압 측정
#define VREF 3.3 // 참조전압
#define MAX_VOLTAGE 4.2 // 리튬이온 배터리의 최대 전압 (충전 시 4.2V) // * 주시는 배터리로 확인
#define MIN_VOLTAGE 3.0 // 리튬이온 배터리의 최소 전압 (방전 시 3.0V)  // * 주시는 배터리로 확인
#define CHARGE_PIN 7 // 충전 회로  제어

// HDC1080
#define HDC_ADDR 0x40 // HDC I2C주소
#define TEMP_REG 0x00
#define HUMIDITY_REG 0x01

#define ADC_PIN 0 // MCP3008 첫 번째 채널
#define WIND_SENSOR_PIN 1 // 풍력
#define SOLAR_SENSOR_PIN 2 // 태양광

// 에너지 부족 임계값
#define ENERGY_LACK 100
// 에너지 부족시 전류 공급
#define POWER_SUPPLY_PIN 4

// OLED 
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// GPIO 핀 정의
#define CURTAIN_SERVO_PIN 4       // 커튼 제어용 서보 모터 핀 (WiringPi 기준) // 16
#define WINDOW_SERVO_PIN 5       // 창문 제어용 서보 모터 핀 (WiringPi 기준) // 18
#define FAN_RELAY_PIN 6          // 팬 제어용 릴레이 핀 (릴레이 채널 1) //22
#define LIGHT_RELAY_PIN 7        // 조명1 제어용 릴레이 핀 (릴레이 채널 2) //26
#define LIGHT2_PIN 18 // 조명 2 제어(릴레이 사용 안함) //18
#define CURTAIN_RELAY_PIN 8      // 커튼 제어용 릴레이 핀 (릴레이 채널 3) //24
#define WINDOW_RELAY_PIN 9       // 창문 제어용 릴레이 핀 (릴레이 채널 4) //21
 
// 임계값 설정
#define LIGHT_THRESHOLD 700      // 조도 임계값 (조정 필요)
#define TEMP_HIGH_THRESHOLD 30   // 높은 온도 임계값 (°C)
#define TEMP_LOW_THRESHOLD 15    // 낮은 온도 임계값 (°C)
#define HUMIDITY_HIGH_THRESHOLD 70 // 높은 습도 임계값 (%)
#define WIND_THRESHOLD 100       // 풍력 에너지 임계값 // * 조정 필요

// 서보 모터 각도 설정 함수
void setServoAngle(int pin, int angle) {
    int pulseWidth = (angle * 2000 / 180) + 500; // 각도를 PWM 신호로 변환 (500~2500us)
    pwmWrite(pin, pulseWidth / 10);             // WiringPi의 pwmWrite 사용
}

// 조도 센서 데이터 읽기
int readLightSensor() {
    int lightValue = mcp3008_read(ADC_PIN);
    return lightValue;
}

//커튼 제어 
void controlCurtain(){
	time_t now;
	struct tm *local;
	time(&now);
	local = localtime(&now);
    int currentHour = local->tm_hour;  // 현재 시각 (24시간 형식)

    // 아침 6시 ~ 10시까지는 커튼을 닫고, 오후 10시 이후에는 커튼을 열기
    if (currentHour >= 6 && currentHour < 10) {
        printf("Close the curtains.\n");
        setServoAngle(CURTAIN_SERVO_PIN, 90); // 커튼 닫기
        relayControl(CURTAIN_RELAY_PIN, HIGH); // 커튼 닫기 (릴레이 사용)
    } else {
        printf("Open the curtains.\n");
        setServoAngle(CURTAIN_SERVO_PIN, 0);  // 커튼 열기
        relayControl(CURTAIN_RELAY_PIN, LOW);  // 커튼 열기 (릴레이 사용)
    }
}

//조명 제어
void controlLED(int lightLevel){
    // 밝기의 정도에 따른 조건 설정
    if (lightLevel > 900) {  // 매우 밝으면
        printf("Turn off both lights.\n");
        relayControl(LIGHT_RELAY_PIN, LOW);   // 첫 번째 조명 끄기
        relayControl(LIGHT2_PIN, LOW); // 두 번째 조명 끄기
    } else if (lightLevel > 500) {  // 중간 밝기이면
        printf("Turn off one light.\n");
        relayControl(LIGHT_RELAY_PIN, LOW);   // 첫 번째 조명 끄기
        relayControl(LIGHT2_PIN, HIGH); // 두 번째 조명 켜기
    } else {  // 어두우면
        printf("Turn on both lights.\n");
        relayControl(LIGHT_RELAY_PIN, HIGH);   // 첫 번째 조명 켜기
        relayControl(LIGHT2_PIN, HIGH); // 두 번째 조명 켜기
    }
}	

// HDC1080에서 온도 데이터를 읽는 함수
float readTemperature(int fd) {
    unsigned int rawTemp = wiringPiI2CReadReg16(fd, TEMP_REG); // 16비트 온도 데이터 읽기
    rawTemp = ((rawTemp >> 8) & 0xFF) | ((rawTemp << 8) & 0xFF00); // 바이트 순서 변경
    float temperature = (rawTemp / 65536.0) * 165.0 - 40.0; // 온도 계산 (0~165°C)
    return temperature;
}

// HDC1080에서 습도 데이터를 읽는 함수
float readHumidity(int fd) {
    unsigned int rawHum = wiringPiI2CReadReg16(fd, HUMIDITY_REG); // 16비트 습도 데이터 읽기
    rawHum = ((rawHum >> 8) & 0xFF) | ((rawHum << 8) & 0xFF00); // 바이트 순서 변경
    float humidity = (rawHum / 65536.0) * 100.0; // 습도 계산 (0~100%)
    return humidity;
}

// 태양광 에너지 데이터 읽기
int readSolarEnergy(){
	int solarEnergyValue = analogRead(SOLAR_SENSOR_PIN);
	return solarEnergyValue;
}

// 풍력 에너지 데이터 읽기
int readWindEnergy() {
    int windEnergyValue = analogRead(WIND_SENSOR_PIN);
    return windEnergyValue;
}

// 배터리 잔량
int BatteryLevel(float voltage){
	if(voltage >= MAX_VOLTAGE){
		return 100;
	}else if (voltage <= MIN_VOLTAGE){
		return 0;
	}else {
		return (int)((voltage - MIN_VOLTAGE) / (MAX_VOLTAGE - MIN_VOLTAGE) * 100);
	}
}

// 충전 회로 시작
void startCharging(){
	digitalWrite(CHARGE_PIN, HIGH);
	printf("Starting Charge\n");
}

// 충전 회로 차단
void stopCharging(){
	digitalWrite(CHARGE_PIN, LOW);
	printf("Charging stop. Battery 4.2 higher\n");
}

//OLED 화면 
void displayEnergyData(int solarEnergy, int windEnergy){
    display.clearDisplay();   // 화면 초기화
    display.setTextSize(1);   // 텍스트 크기 설정
    display.setTextColor(SSD1306_WHITE); // 텍스트 색상 설정

   char solarStr[10], windStr[10];
   sprintf(solarStr, "%d", solarEnergy);
   sprintf(windStr, "%d", windEnergy);

    // 첫 번째 줄에 태양광 전압 표시
    display.setCursor(0, 0);   
    display.print("Solar: "); 
    display.print(solarStr); // 태양광 에너지 값 표시
    display.print(" V");
    // 두 번째 줄에 풍력 전압 표시
    display.setCursor(0, 10);
    display.print("Wind: "); 
    display.print(windStr); // 풍력 에너지 값 표시
    display.print(" V");

    display.display(); // OLED에 표시
}
	
//블루투스 연결
int setupBluetoothServer() {
    struct sockaddr_rc addr = { 0 };
    int sock, client;
    char buf[256];

    // RFCOMM 소켓 열기
    sock = socket(AF_BLUETOOTH, SOCK_STREAM, BTPROTO_RFCOMM);
    if (sock == -1) {
        perror("Socket creation failed");
        return -1;
    }

    addr.rc_family = AF_BLUETOOTH;
    addr.rc_bdaddr = *BDADDR_ANY;
    addr.rc_channel = (uint8_t) 1;

    // 서버 주소 바인딩
    if (bind(sock, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        perror("Bind failed");
        return -1;
    }

    // 클라이언트 연결 대기
    if (listen(sock, 1) < 0) {
        perror("Listen failed");
        return -1;
    }

    printf("Waiting for Bluetooth connection...\n");
    client = accept(sock, NULL, NULL);
    if (client < 0) {
        perror("Accept failed");
        return -1;
    }

    printf("Bluetooth connected\n");

    return client;
}

void controlByBluetooth(int client) {
    char buf[256];
    int bytes_read;

    // 블루투스 명령 수신
    bytes_read = read(client, buf, sizeof(buf));
    int bytes_read = read(client, buf, sizeof(buf));

    if (bytes_read > 0) {
        buf[bytes_read] = '\0'; // null terminator 추가
        cleanInput(buf);        // 공백 제거
        printf("Received command: %s\n", buf);


        if (strcmp(buf, "close window") == 0) {
            printf("Closing window\n");
            setServoAngle(WINDOW_SERVO_PIN, 0);  // 창문 닫기
            relayControl(WINDOW_RELAY_PIN, LOW); // 창문 닫기 (릴레이 사용)
        }else if (strcmp(buf, "open window") == 0) {
            printf("Closing curtain\n");
            setServoAngle(WINDOW_SERVO_PIN, 90); // 창문 열기
            relayControl(WINDOW_RELAY_PIN, HIGH); // 창문 열기 (릴레이 사용)
        } 
        else if (strcmp(buf, "close curtain") == 0) {
            printf("Closing curtain\n");
            setServoAngle(CURTAIN_SERVO_PIN, 90); // 커튼 닫기
            relayControl(CURTAIN_RELAY_PIN, HIGH); // 커튼 닫기 (릴레이 사용)
        }else if (strcmp(buf, "open curtain") == 0) {
            printf("Closing curtain\n");
            setServoAngle(CURTAIN_SERVO_PIN, 0); // 커튼 열기
            relayControl(CURTAIN_RELAY_PIN, LOW); // 커튼 열기 (릴레이 사용)
        } 
        else if (strcmp(buf, "turn on led1") == 0) {
            printf("Turning on LED 1\n");
            relayControl(LIGHT_RELAY_PIN, HIGH);   // 첫 번째 조명 켜기
        }else if (strcmp(buf, "turn off led1") == 0) {
            printf("Turning off LED 1\n");
            relayControl(LIGHT2_PIN, LOW); // 첫 번째 조명 끄기
        }
        else if (strcmp(buf, "turn off led2") == 0) {
            printf("Turning off LED 2\n");
            relayControl(LIGHT2_PIN, LOW); // 두 번째 조명 끄기
        }
        else if (strcmp(buf, "turn on led2") == 0) {
            printf("Turning on LED 2\n");
            relayControl(LIGHT_RELAY_PIN, HIGH);   // 첫 번째 조명 켜기
        }
        else {
            printf("Unknown command\n");
        }
    }
}
//블루투스 전송
void sendDataToBluetooth(int client, float temperature, float humidity, int windEnergy, int solarEnergy) {
    char buffer[256];
    // 센서 데이터를 문자열 형식으로 포맷
    snprintf(buffer, sizeof(buffer), "TEMP:%.2f,HUM:%.2f,WIND:%d,SOLAR:%d\n", temperature, humidity, windEnergy, solarEnergy);

    // 블루투스 클라이언트로 데이터 전송
    write(client, buffer, strlen(buffer));
}


// 릴레이 제어 함수
void relayControl(int pin, int state) {
    digitalWrite(pin, state);  // HIGH: 켜기, LOW: 끄기
}

//에너지 부족시 전력
void checkEnergy(int solarEnergy, int windEnergy){
	if(solarEnergy < ENERGY_LACK && windEnergy < ENERGY_LACK){
		printf("Lack of Energy -> raspberryPi current supply\n");
		relayControl(POWER_SUPPLY_PIN, HIGH);
	}else{
		relayControl(POWER_SUPPLY_PIN, LOW);
	}
}

int main() {
    // HDC센서, I2C장치 열기
    int fd = wiringPiI2CSetup(HDC_ADDR);
    if (fd == -1){
        printf("WiringPi setup failed\n");
        return 1;
    }
    // WiringPi 초기화
    if (wiringPiSetup() == -1) {
        printf("WiringPi setup failed\n");
        return 1;
    }   
    // 블루투스 서버 설정
    int client = setupBluetoothServer();
    if (client == -1) {
        return 1; // 블루투스 연결 실패시 종료
    }
void cleanInput(char *str) {
    char *p = str;
    while (*p != '\0') {
        if (*p == '\r' || *p == '\n') *p = '\0';  // 개행 문자 제거
        p++;
    }
}

    // GPIO 핀 설정
    pinMode(CURTAIN_SERVO_PIN, PWM_OUTPUT);
    pinMode(WINDOW_SERVO_PIN, PWM_OUTPUT);
    pinMode(FAN_RELAY_PIN, OUTPUT);
    pinMode(LIGHT_RELAY_PIN, OUTPUT);
    pinMode(CURTAIN_RELAY_PIN, OUTPUT);
    pinMode(WINDOW_RELAY_PIN, OUTPUT);
    pinMode(CHARGE_PIN, OUTPUT);
    digitalWrite(CHARGE_PIN, HIGH);//기본적으로 활성화된 충전회로

    // PWM 설정
    pwmSetMode(PWM_MODE_MS);
    pwmSetRange(2000);
    pwmSetClock(192);
	
	//mcp3008 초기화
	mcp3008Setup(100,0);
	// OLED 화면 초기화
	if(!display.begin(SSD1306_I2C_ADDRESS, OLED_RESET)){
		printf("OLED init failed\n");
		return 1;
	} display.display();

    printf("Smart Home Control System Starting!\n");

    while (1) {
        // 센서 데이터 읽기
        int lightLevel = readLightSensor();
        float temperature = readTemperature(fd);
        float humidity = readHumidity(fd);
	int solarEnergy = readSolarEnergy();
        int windEnergy = readWindEnergy();
	int adcValue = analogRead(ADC_PIN); 
	float voltage = (adcValue * VREF) / 1024.0;
	int batteryLevel = BatteryLevel(voltage);

    struct timeval tv;
    fd_set readfds;

    FD_ZERO(&readfds);
    FD_SET(client, &readfds);

    tv.tv_sec = 5; // 5초 대기
    tv.tv_usec = 0;

    int activity = select(client + 1, &readfds, NULL, NULL, &tv);
    if (activity > 0 && FD_ISSET(client, &readfds)) {
        controlByBluetooth(client);
    }
 	//OLED 화면 업데이트
	displayEnergyData(solarEnergy, windEnergy);

	// 배터리 전압이 4.2이면 충전 멈춤
	if(voltage >= MAX_VOLTAGE){
		stopCharging();	
		charging = 0;
	}else{
		startCharging();
		charging = 1;
	}

	// 커튼 제어
	controlCurtain();

	//LED 제어 
	controlLED(lightlevel);
 	
	// 에너지 부족시 전력 공급
	checkEnergy(solarEnergy, windEnergy);


        //  온도 및 습도에 따른 창문 및 팬 제어
        if (temperature > TEMP_HIGH_THRESHOLD || humidity > HUMIDITY_HIGH_THRESHOLD) {
            printf("Temp and Humidity high. open the window , turn on the fan. \n");
            setServoAngle(WINDOW_SERVO_PIN, 90); // 창문 열기
            relayControl(FAN_RELAY_PIN, HIGH);   // 팬 켬
            relayControl(WINDOW_RELAY_PIN, HIGH); // 창문 열기 (릴레이 사용)
        } else if (temperature < TEMP_LOW_THRESHOLD) {
            printf("Temp and Humidity low. close the window, turn off the fan.\n");
            setServoAngle(WINDOW_SERVO_PIN, 0);  // 창문 닫기
            relayControl(FAN_RELAY_PIN, LOW);    // 팬 끔
            relayControl(WINDOW_RELAY_PIN, LOW); // 창문 닫기 (릴레이 사용)
        } else {
            setServoAngle(WINDOW_SERVO_PIN, 0);  // 창문 닫기
            relayControl(FAN_RELAY_PIN, LOW);    // 팬 끔
            relayControl(WINDOW_RELAY_PIN, LOW); // 창문 닫기 (릴레이 사용)
        }

        // 풍력 에너지가 강한 경우 창문 닫기
        if (windEnergy > WIND_THRESHOLD) {
            printf("Strong wind. Close the window.\n");
            setServoAngle(WINDOW_SERVO_PIN, 0); // 창문 닫기
            relayControl(WINDOW_RELAY_PIN, LOW); // 창문 닫기 (릴레이 사용)
        }

    sendDataToBluetooth(client, temperature, humidity, windEnergy, solarEnergy);
        delay(5000); // 5초 간격으로 반복
    }

    return 0;
}
