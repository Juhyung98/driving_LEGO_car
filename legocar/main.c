#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>
#include "gattlib.h"
#include <signal.h>
#include <sys/types.h>
#include <unistd.h>
#include <termios.h>
#include <time.h>

#define TRUE	1
#define FALSE	0

#define CONTROL_PLUS_MAC	"90:84:2B:4C:84:8A"					// Control+ hub의 mac 주소
#define CONTROL_PLUS_UUID	"00001624-1212-efde-1623-785feabcd123"	// hub에서 지원하는 서비스 UUID

#define ROTATION_SPEED	20		// 모터 회전 속도. 최대 100

typedef enum { READ, WRITE } operation_t;
operation_t g_operation;

typedef enum {
	NAME =	0x00,
	RAW =	0x01,
	PCT =	0x02,
	SI = 	0x03,
	SYMBOL =	0x04,
	MAPPING =	0x05,
	INTERNAL = 	0x06,
	BIAS = 	0x07,
	CAPABILITY = 	0x03,
	FORMAT =	0x80
} information_t;

typedef enum {
	LEFT,
	RIGHT
} direction_t;


static uuid_t g_uuid;
long int value_data;
gatt_connection_t* connection = NULL;
void* adapter;
int connected = FALSE;
int read_req = FALSE;

char write_buf[5000];

struct timespec rqtp, rmtp;

static pthread_mutex_t g_mutex = PTHREAD_MUTEX_INITIALIZER;

static void recv_data();
void send_error(uuid_t g_uuid);
int send_basic_motor_speed(int port, int speed);

int getch()
{
	int c;
	struct termios oldattr, newattr;

	tcgetattr(STDIN_FILENO, &oldattr);           // 현재 터미널 설정 읽음
	newattr = oldattr;
	newattr.c_lflag &= ~(ICANON | ECHO);         // CANONICAL과 ECHO 끔
	newattr.c_cc[VMIN] = 1;                      // 최소 입력 문자 수를 1로 설정
	newattr.c_cc[VTIME] = 0;                     // 최소 읽기 대기 시간을 0으로 설정
	tcsetattr(STDIN_FILENO, TCSANOW, &newattr);  // 터미널에 설정 입력
	c = getchar();                               // 키보드 입력 읽음
	tcsetattr(STDIN_FILENO, TCSANOW, &oldattr);  // 원래의 설정으로 복구
	return c;
}

void send_error(uuid_t g_uuid)
{
	int ret;
	char uuid_str[MAX_LEN_UUID_STR + 1]; 

	gattlib_uuid_to_string(&g_uuid, uuid_str, sizeof(uuid_str));

	if (ret == GATTLIB_NOT_FOUND) {
		fprintf(stderr, "Could not find GATT Characteristic with UUID %s. "
				"You might call the program with '--gatt-discovery'.\n", uuid_str);
	} else {
		fprintf(stderr, "Error while writing GATT Characteristic with UUID %s (ret:%d)\n",
				uuid_str, ret);
	}   

	gattlib_disconnect(connection);
	exit(1);
}


// function to order same command
void move_speed(int speed)
{
	send_basic_motor_speed(0, speed);
	send_basic_motor_speed(1, speed);
}

// 앞바퀴 방향 회전.
// motor moves forever (but, it couldn't)
// if use nanosleep(), motor moves little (not forever)
// port 2 : rotate motor
					// port 2
// send_basic_motor_speed(2, ~)
void rotate(direction_t d)
{
	if(d == LEFT) {
		send_basic_motor_speed(2, ROTATION_SPEED);
	}
	else if(d == RIGHT) {
		send_basic_motor_speed(2, -ROTATION_SPEED);
	}
	nanosleep(&rqtp, &rmtp);
	send_basic_motor_speed(2, 0);
}


// 모터의 스피드 결정 -100 ~ 100 사이의 값
// port : 포트번호 앞바퀴 0, 뒷바퀴 1, 회전 2
// port 0 and port 1 have to get same command --> move_speed(speed);
// speed : -100 ~ 100. 음수는 반대로 움직임
int send_basic_motor_speed(int port, int speed)
{
	int ret;
	int len = 8;

	speed = -speed;

	if(speed < 0) {
		speed = 256 + speed;
	}

	// order using byte code
	// make one byte for each and order using bluetooth
	write_buf[0] = len;			// length
	write_buf[1] = 0x00;
	write_buf[2] = 0x81;		// port input format setup (single)
	write_buf[3] = port;		// port no. (motor no)
	write_buf[4] = 0x11;		// execute immediately & command feedback (status)
	write_buf[5] = 0x51;
	write_buf[6] = 0x00;
	write_buf[7] = speed; 		// speed of motor (max speed is 100)

	
	// send write_buf to g_uuid
	// which motor, which speed
	ret = gattlib_write_char_by_uuid(connection, &g_uuid, write_buf, len);
	if (ret != GATTLIB_SUCCESS) {
		send_error(g_uuid);
	}  

//	sleep(1);
//	recv_data();			// 응답을 받을 필요가 있을 때

	return len;
}

int send_port_input_format_setup_single(int port, information_t type)
{
	int ret;
	int len = 10;

	write_buf[0] = len;			// length
	write_buf[1] = 0x00;
	write_buf[2] = 0x41;		// port input format setup (single)
	write_buf[3] = port;		// port no.
	write_buf[4] = 0x00;		// ?, The Mode to get information for
	write_buf[5] = type;


	ret = gattlib_write_char_by_uuid(connection, &g_uuid, write_buf, len);
	if (ret != GATTLIB_SUCCESS) {
		send_error(g_uuid);
	}  

//	sleep(1);
	recv_data();

	return len;
}

int send_port_mode_information_request(int port, information_t type)
{
	int ret;

	write_buf[0] = 0x06;		// length
	write_buf[1] = 0x00;
	write_buf[2] = 0x22;		// port information request message
	write_buf[3] = port;		// port no.
	write_buf[4] = 0x00;		// ?, The Mode to get information for
	write_buf[5] = type;


	ret = gattlib_write_char_by_uuid(connection, &g_uuid, write_buf, 6);
	if (ret != GATTLIB_SUCCESS) {
		send_error(g_uuid);
	}  

//	sleep(1);
	recv_data();

	return 6;
}

int send_port_information_request(int port)
{
	int ret;

	write_buf[0] = 0x05;		// length
	write_buf[1] = 0x00;
	write_buf[2] = 0x21;		// port information request message
	write_buf[3] = port;		// port no.
	write_buf[4] = 0x02;		// 0x00 port value, 0x01 mode info, 0x02 possible mode combinations

	ret = gattlib_write_char_by_uuid(connection, &g_uuid, write_buf, 5);
	if (ret != GATTLIB_SUCCESS) {
		send_error(g_uuid);
	}  

//	sleep(1);
	recv_data();

	return 5;
}

// 데이터를 받기위한 시그널을 보냄
static void recv_data() {
	kill(0, SIGUSR1);
}

// 허브로부터 데이터 받음. SIGUSR1 시그널 발생시 실행
static void _recv_data() {
	uint8_t *buffer = NULL;
	int ret;
	int len;
	int msg_len;

	printf("SIGNAL!\n");

	if(!connection) {
		fprintf(stderr, "ERROR: Connection is not established.\n");
		exit(1);
	}

	ret = gattlib_read_char_by_uuid(connection, &g_uuid, (void **)&buffer, &len);
	if (ret != GATTLIB_SUCCESS) {
		char uuid_str[MAX_LEN_UUID_STR + 1]; 

		gattlib_uuid_to_string(&g_uuid, uuid_str, sizeof(uuid_str));

		if (ret == GATTLIB_NOT_FOUND) {
			fprintf(stderr, "Could not find GATT Characteristic with UUID %s. "
					"You might call the program with '--gatt-discovery'.\n", uuid_str);
		} else {
			fprintf(stderr, "Error while reading GATT Characteristic with UUID %s (ret:%d)\n", uuid_str, ret);
		}   
		gattlib_disconnect(connection);
		exit(1);
	}   
	msg_len = (int)buffer[0];
	
	if(len < 3) {
		fprintf(stderr, "Message length is short.");
		gattlib_disconnect(connection);
		exit(1);
	}

	printf("Read UUID completed: ");
	for (int i = 0; i < msg_len; i++) {
		printf("%02x ", buffer[i]);
	}   
	printf("\n");

	free(buffer);
	return;
}

int main(int argc, char *argv[])
{
	int c, speed = 0;
	int ret;
	const char* adapter_name;
	pthread_t thread;
	int retry_count = 5;

	// rqtp, rmtp 설정. 방향전환을 위해 필요함
	rqtp.tv_sec = 0;
	rqtp.tv_nsec = 10000000;
	rmtp.tv_sec = 0;
	rmtp.tv_nsec = 0;

	// 데이터를 받기위해 필요함. 시그널 발생시에 데이터 받음
	signal(SIGUSR1, _recv_data);


	// 블루투스 동작
	ret = gattlib_adapter_open(adapter_name, &adapter);
	if (ret) {
		fprintf(stderr, "ERROR: Failed to open adapter.\n");
		return 1;
	}  

	// UUID 설정
	// convert string to uuid
	// do it once
	if (gattlib_string_to_uuid(CONTROL_PLUS_UUID, strlen(CONTROL_PLUS_UUID) + 1, &g_uuid) < 0) {
		fprintf(stderr, "Error: Cannot translate UUID (%s)\n", CONTROL_PLUS_UUID);
		return 1;
	}   

	// 블루투스 접속
	// gattlib_connect() --> connect
	// failed sometimes
	// do it while non-connection 
	while(retry_count > 0) {
		connection = gattlib_connect(NULL, CONTROL_PLUS_MAC, GATTLIB_CONNECTION_OPTIONS_LEGACY_DEFAULT);
		if (connection == NULL) {
			fprintf(stderr, "Fail to connect to the bluetooth device.\n");
			retry_count--;
			// Max 10 loop
			sleep(1);
			continue;
		}
		else {
			break;
		}
	}
	if(connection == NULL) {
		exit(1);
	}

	printf("Connected\n");
	///////////////////////////////////////////////////////////////
	while(1) {
		c = getch();
		switch(c) {
			case 'q':
				goto EXIT;
				break;

			case 'w':
				speed += 10;
				if(speed > 100) speed = 100;
				move_speed(speed);
				break;

			case 's':
				speed -= 10;
				if(speed < -100) speed = -100;
				move_speed(speed);
				break;

			case 'a':
				rotate(LEFT);
				break;
				
			case 'd':
				rotate(RIGHT);
				break;

			case 'b':
				speed = 0;
				move_speed(speed);
				break;
			default:
				break;
		}
	}
//	move_speed(50);
//	sleep(1);
//	move_speed(0);
	

	///////////////////////////////////////////////////////////////
EXIT:
	gattlib_disconnect(connection);
//	gattlib_adapter_close(adapter);
	return 0;
}

