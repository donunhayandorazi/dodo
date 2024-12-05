#include <stdio.h>
#include <stdlib.h>
#include <wiringPi.h>
#include <wiringPiI2C.h>
#include <unistd.h>
#include <time.h>
#include <mcp3008.h> // ADC 아날로그 변환기 // 교수님 // 리튬이온배터리
#include <Adafruit_SSD1306.h>
#include <Adafruit_GFX.h>
#include <bluetooth/bluetooth.h>
#include <bluetooth/rfcomm.h>
#include <bluetooth/sco.h>

#define SERIAL_PORT "/dev/ttyAMA0"  // 블루투스 모듈이 연결된 포트

// ET-Board에 연결된 핀 정의 (WiringPi 기준)
#define WIND_SENSOR_PIN 15 // 풍력 센서 입력 핀 A3
#define SOLAR_SENSOR_PIN 19 // 태양광 센서 입력 핀 A5
#define OLED_RESET -1

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// HDC1080 (온습도 센서)
#define HDC_ADDR 0x40
#define TEMP_REG 0x00
#define HUMIDITY_REG 0x01

// 배터리 전압 측정 (시스템 상태)
#define VREF 3.3 // 참조전압
#define MAX_VOLTAGE 4.2 // 리튬이온 배터리 최대 전압 (4.2V)
#define MIN_VOLTAGE 3.0 // 리튬이온 배터리 최소 전압 (3.0V)
#define CHARGE_PIN 7 // 충전 회로 제어 핀

// 에너지 부족 임계값
#define ENERGY_LACK 100
// 에너지 부족 시 전류 공급
#define POWER_SUPPLY_PIN 4

// GPIO 핀 정의
#define CURTAIN_SERVO_PIN 4 // 7
#define WINDOW_SERVO_PIN 5 // 29
#define FAN_RELAY_PIN 6 // 31
#define LIGHT_RELAY_PIN 7 //26
#define LIGHT2_PIN 18 //12
#define CURTAIN_RELAY_PIN 8 //24
#define WINDOW_RELAY_PIN 9 //21

#define LIGHT_THRESHOLD 700 // 조도 임계값 (조정 필요)
#define TEMP_HIGH_THRESHOLD 30 // 높은 온도 임계값 (°C)
#define TEMP_LOW_THRESHOLD 15  // 낮은 온도 임계값 (°C)
#define HUMIDITY_HIGH_THRESHOLD 70 // 높은 습도 임계값 (%)
#define WIND_THRESHOLD 100 // 풍력 에너지 임계값 // * 조정 필요

// 서보 모터 각도 설정 함수
void setServoAngle(int pin, int angle) {
    int pulseWidth = (angle * 2000 / 180) + 500; 
    pwmWrite(pin, pulseWidth / 10);
}

// 조도 센서 데이터 읽기
int readLightSensor() {
    int lightValue = mcp3008_read(ADC_PIN);
    return lightValue;
}

//조명 제어
void controlLED(int lightLevel){
    // 밝기의 정도에 따른 조건 설정
    if (lightLevel > 900) {  // 매우 
        printf("Turn off both lights.\n");
        relayControl(LIGHT_RELAY_PIN, LOW);   // 첫 번째 조명 끄기
        relayControl(LIGHT2_PIN, LOW); // 두 번째 조명 끄기
    } else if (lightLevel > 500) {  // 중간 밝기
        printf("Turn off one light.\n");
        relayControl(LIGHT_RELAY_PIN, LOW);   // 첫 번째 조명 끄기
        relayControl(LIGHT2_PIN, HIGH); // 두 번째 조명 켜기
    } else {  // 어두움
        printf("Turn on both lights.\n");
        relayControl(LIGHT_RELAY_PIN, HIGH);   // 첫 번째 조명 켜기
        relayControl(LIGHT2_PIN, HIGH); // 두 번째 조명 켜기
    }
}	

//커튼 제어 
void controlCurtain(){
	time_t now;
	struct tm *local;
	time(&now);
	local = localtime(&now);
    int currentHour = local->tm_hour;  // 현재 시각 (24시간 형식)

    // 아침 6시 ~ 10시까지는 커튼을 닫고, 오전 10시 이후에는 커튼을 열기
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


// 풍력 에너지 데이터 읽기
int readWindEnergy() {
    int windEnergyValue = analogRead(WIND_SENSOR_PIN);  // ET-Board의 풍력 센서에서 값 읽기
    return windEnergyValue;
}

// 태양광 에너지 데이터 읽기
int readSolarEnergy() {
    int solarEnergyValue = analogRead(SOLAR_SENSOR_PIN);  // ET-Board의 태양광 센서에서 값 읽기
    return solarEnergyValue;
}

// OLED 화면에 에너지 데이터 표시
void displayEnergyData(int solarEnergy, int windEnergy) {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);

    char solarStr[10], windStr[10];
    sprintf(solarStr, "%d", solarEnergy);
    sprintf(windStr, "%d", windEnergy);

    display.setCursor(0, 0);   
    display.print("Solar: "); 
    display.print(solarStr);
    display.print(" V");

    display.setCursor(0, 10);
    display.print("Wind: "); 
    display.print(windStr);
    display.print(" V");

    display.display();
}

// HDC1080에서 온도 데이터를 읽는 함수
float readTemperature(int fd) {
    unsigned int rawTemp = wiringPiI2CReadReg16(fd, TEMP_REG);
    rawTemp = ((rawTemp >> 8) & 0xFF) | ((rawTemp << 8) & 0xFF00);
    float temperature = (rawTemp / 65536.0) * 165.0 - 40.0;
    return temperature;
}
// HDC1080에서 습도 데이터를 읽는 함수
float readHumidity(int fd) {
    unsigned int rawHum = wiringPiI2CReadReg16(fd, HUMIDITY_REG);
    rawHum = ((rawHum >> 8) & 0xFF) | ((rawHum << 8) & 0xFF00);
    float humidity = (rawHum / 65536.0) * 100.0;
    return humidity;
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

//에너지 부족시 전력
void checkEnergy(int solarEnergy, int windEnergy){
	if(solarEnergy < ENERGY_LACK && windEnergy < ENERGY_LACK){
		printf("Lack of Energy -> raspberryPi current supply\n");
		relayControl(POWER_SUPPLY_PIN, HIGH);
	}else{
		relayControl(POWER_SUPPLY_PIN, LOW);
	}
}

// 블루투스 연결 설정
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

// 연결 대기
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

    printf("Client connected\n");
    close(sock);
    return client;
}

// 블루투스 데이터 처리
void controlByBluetooth(int client) {
    char buf[256];
    int bytes_read;
    // 명령 수신
    bytes_read = read(client, buf, sizeof(buf));
    if (bytes_read > 0) {
        buf[bytes_read] = '\0';
        printf("Received Bluetooth data: %s\n", buf);

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

// 메인 함수
int main(void) {
    int fd = wiringPiI2CSetup(HDC_ADDR);
    
    wiringPiSetup(); // WiringPi 라이브러리 초기화
    mcp3008Setup(100, 0); // MCP3008 설정

    // PWM 설정
    pwmSetMode(PWM_MODE_MS);
    pwmSetRange(2000);
    pwmSetClock(192);

    // HDC1080 I2C 초기화
    if (fd == -1) {
        printf("Failed to initialize I2C\n");
        return -1;
    }
    // WiringPi 초기화
    if (wiringPiSetup() == -1) {
        printf("WiringPi setup failed\n");
        return 1;
    }   

    display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // OLED 초기화
    display.display();

    // 블루투스 서버 설정
    int client = setupBluetoothServer();
    if (client < 0) {
        printf("Bluetooth setup failed\n");
        return -1;
    }

    // GPIO 핀 설정
    pinMode(CURTAIN_SERVO_PIN, PWM_OUTPUT);
    pinMode(WINDOW_SERVO_PIN, PWM_OUTPUT);
    pinMode(FAN_RELAY_PIN, OUTPUT);
    pinMode(LIGHT_RELAY_PIN, OUTPUT);
    pinMode(CURTAIN_RELAY_PIN, OUTPUT);
    pinMode(WINDOW_RELAY_PIN, OUTPUT);
    pinMode(CHARGE_PIN, OUTPUT);
    digitalWrite(CHARGE_PIN, HIGH); //기본적으로 활성화된 충전회로

    printf("Smart Home Control System Starting!\n");

    while(1) {
        // 센서 데이터 읽기
        int lightLevel = readLightSensor();
        float temperature = readTemperature(fd);
        float humidity = readHumidity(fd);
        int solarEnergy = readSolarEnergy();
        int windEnergy = readWindEnergy();
        int adcValue = analogRead(ADC_PIN); 
        float voltage = (adcValue * VREF) / 1024.0;
        int batteryLevel = BatteryLevel(voltage);

        // OLED 화면에 에너지 데이터 출력
        displayEnergyData(solarEnergy, windEnergy);

        // 커튼 제어
        controlCurtain();

        //LED 제어 
        controlLED(lightlevel);

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

        // 에너지 부족시 전력 공급
        checkEnergy(solarEnergy, windEnergy);


        // 블루투스 명령 처리
        controlByBluetooth(client);
        sendDataToBluetooth(client, temperature, humidity, windEnergy, solarEnergy);

        delay(5000); // 5초마다 데이터 업데이트
    }

    close(client);
    return 0;
}
