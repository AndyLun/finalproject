#include "mbed.h"
#include "bbcar.h"

DigitalOut rLED(LED1);
DigitalOut gLED(LED2);
DigitalOut bLED(LED3);

Serial pc(USBTX, USBRX);
Serial camera(D1, D0);
RawSerial xbee(D12, D11);
DigitalInOut ping(D9);
Timer t;

EventQueue queue_xbee(32 * EVENTS_EVENT_SIZE);
Thread th_xbee;
Thread th_logger;

Ticker servo_ticker;
PwmOut pinSVL(D10), pinSVR(D13);
BBCar car(pinSVL, pinSVR, servo_ticker);

Ticker encoder0_ticker;
Ticker encoder1_ticker;
DigitalIn pinEL(D4), pinER(D5);
parallax_encoder encoder0(pinEL, encoder0_ticker);
parallax_encoder encoder1(pinER, encoder1_ticker);

char imageid = 'x';
int step = 0;
float pval = 0;

void xbee_rx_interrupt(void);
void xbee_rx(void);
void reply_messange(char *xbee_reply, char *messange);
void check_addr(char *xbee_reply, char *messenger);
float getPing();
void log();

void log() {
	while(true) {
		if(step >= 28) break;
		xbee.printf("Step: %d\r\n", step);
		wait(1);
	}
	xbee.printf("Finished!\r\n");
}

int main() {
	pc.baud(9600);
	camera.baud(9600);
	rLED = 0; gLED = 1; bLED = 1;

	// XBee setting
	
	char xbee_reply[4];
	xbee.baud(9600);
	xbee.printf("+++");
	xbee_reply[0] = xbee.getc();
	xbee_reply[1] = xbee.getc();
	if (xbee_reply[0] == 'O' && xbee_reply[1] == 'K')
	{
		pc.printf("enter AT mode.\r\n");
		xbee_reply[0] = '\0';
		xbee_reply[1] = '\0';
	}
	xbee.printf("ATMY 0x670\r\n");
	reply_messange(xbee_reply, "setting MY : 0x670");
	xbee.printf("ATDL 0x770\r\n");
	reply_messange(xbee_reply, "setting DL : 0x770");
	xbee.printf("ATID 0x17\r\n");
	reply_messange(xbee_reply, "setting PAN ID : 0x17");
	xbee.printf("ATWR\r\n");
	reply_messange(xbee_reply, "write config");
	xbee.printf("ATMY\r\n");
	check_addr(xbee_reply, "MY");
	xbee.printf("ATDL\r\n");
	check_addr(xbee_reply, "DL");
	xbee.printf("ATCN\r\n");
	reply_messange(xbee_reply, "exit AT mode");
	xbee.getc();

	// start
	pc.printf("start\r\n");
	th_xbee.start(callback(&queue_xbee, &EventQueue::dispatch_forever));

	// Setup a serial interrupt function of receiving data from xbee
	xbee.attach(xbee_rx_interrupt, Serial::RxIrq);

	/////
	rLED = 1; gLED = 0;
	xbee.printf("Start\n");
	car.stop();

	th_logger.start(log);

	// 1 : Long straight
	step = 1;
	do {
		pval = getPing();
		car.goStraight(50);
		wait(0.1);
	} while(pval > 23);
	car.stop();

	// 2 : Left turn on data matrix
	step = 2;
	encoder1.reset();
	car.setRightSpeed(50);
	while(encoder1.get_steps() < 28) wait_ms(50);
	car.stop();

	// 3 : Go straight to end
	step = 3;
	encoder0.reset();
	car.goStraight(50);
	while(encoder0.get_cm() < 50) wait_ms(50);
	do {
		pval = getPing();
		car.goStraight(50);
		wait(0.1);
	} while(pval > 8);
	car.stop();

	// 4 : Turn to image
	step = 4;
	encoder1.reset();
	car.setRightSpeed(-40);
	while(encoder1.get_steps() < 26) wait_ms(50);
	car.stop();

	// 5 : Reverse to get proper view
	step = 5;
	encoder0.reset();
	car.goStraight(-50);
	while(encoder0.get_cm() < 7) wait_ms(50);
	car.stop();

	// 6 : Take picture + 7: Image classification
	step = 6;
	wait(1);
	char s[6];
	sprintf(s, "snap");
	camera.puts(s);
	pc.printf("Snapped\r\n");
	wait(4);
	if(camera.readable()) {
		imageid = camera.getc();
		xbee.printf("Image: %c", imageid);
	}

	// 8 : Move forward again
	step = 8;
	encoder0.reset();
	car.goStraight(50);
	while(encoder0.get_cm() < 7) wait_ms(50);
	car.stop();

	// 9 : Turn to face wall again
	step = 9;
	encoder1.reset();
	car.setRightSpeed(40);
	while(encoder1.get_steps() < 27) wait_ms(50);
	car.stop();

	// 10 : Reverse to parking spot
	step = 10;
	encoder0.reset();
	car.goStraight(-50);
	while(encoder0.get_cm() < 51) wait_ms(50);
	car.stop();

	// 11 : Turn to parking spot
	step = 11;
	encoder1.reset();
	car.setRightSpeed(-40);
	while(encoder1.get_steps() < 28) wait_ms(50);
	car.stop();

	// 12 : Parking
	step = 12;
	encoder0.reset();
	car.goStraight(-50);
	while(encoder0.get_cm() < 30) wait_ms(50);
	car.stop();

	wait(1);

	// 13 : Come out of parking spot
	step = 13;
	encoder0.reset();
	car.goStraight(50);
	while(encoder0.get_cm() < 31) wait_ms(50);
	car.stop();

	// 14 : Turn right (1/3 180 deg turn)
	step = 14;
	encoder0.reset();
	car.setLeftSpeed(50);
	while(encoder0.get_steps() < 28) wait_ms(50);
	car.stop();
	
	// 15 : Move forward (2/3 180 deg)
	step = 15;
	encoder0.reset();
	car.goStraight(50);
	while(encoder0.get_cm() < 9) wait_ms(50);
	car.stop();

	// 16 : Turn right (3/3 180 deg)
	step = 16;
	encoder0.reset();
	car.setLeftSpeed(50);
	while(encoder0.get_steps() < 26) wait_ms(50);
	car.stop();

	// 17 : Long straight to mission 2
	step = 17;
	encoder0.reset();
	car.goStraight(50);
	while(encoder0.get_cm() < 60) wait_ms(50);
	do {
		pval = getPing();
		wait(0.1);
	} while (pval > 21);
	car.stop();
	

	// 18 : Turn right to mission 2 (1/3 180)
	step = 18;
	encoder0.reset();
	car.setLeftSpeed(50);
	while(encoder0.get_steps() < 26) wait_ms(50);
	car.stop();

	// 19 : Forward (2/3 180)
	step = 19;
	encoder0.reset();
	car.goStraight(50);
	while(encoder0.get_cm() < 11) wait_ms(50);
	car.stop();

	// 20 : Turn right (3/3 180)
	step = 20;
	encoder0.reset();
	car.setLeftSpeed(50);
	while(encoder0.get_steps() < 26) wait_ms(50);
	car.stop();

	// 21 : Move forward to scan object
	step = 21;
	encoder0.reset();
	car.goStraight(50);
	while(encoder0.get_cm() < 25) wait_ms(50);
	car.stop();
	
	// 22 : Scan object
	step = 22;
	float pstr = 0, pleft = 0, pright = 0;

	for(int i = 0; i < 5; i++) {
		pval = getPing();
		pstr += pval;
		xbee.printf("i.Ping = %lf\r\n", pval);
		wait(0.1);
	}
	car.setLeftSpeed(-15);
	wait(0.6);
	car.stop();
	for (int i = 0; i < 5; i++)
	{
		pval = getPing();
		pleft += pval;
		xbee.printf("ii.Ping = %lf\r\n", pval);
		wait(0.1);
	}
	car.setLeftSpeed(15);
	wait(1.2);
	car.stop();
	for (int i = 0; i < 5; i++)
	{
		pval = getPing();
		pright += pval;
		xbee.printf("iii.Ping = %lf\r\n", pval);
		wait(0.1);
	}
	pstr /= 5; pleft /= 5; pright /= 5;

	if((pright - pleft) > 1) {
		xbee.printf("Object: SKEW\r\n");
	} else if((abs(pleft - pright) < 1.5) && (abs(pleft - pstr) < 1.5)) {
		if(pstr > 24) xbee.printf("Object: HOLLOW\r\n");
		else xbee.printf("Object: SKEW\r\n");
	} else {
		if(pstr < pleft) {
			if(pstr > 15) xbee.printf("Object: TRIANGLE\r\n");
			else xbee.printf("Object: SQUARE\r\n");
		} else xbee.printf("Object: UNKNOWN\r\n");
	}

	car.setLeftSpeed(-15);
	wait(0.6);
	car.stop();

	// 23 : Reverse
	step = 23;
	encoder0.reset();
	car.goStraight(-50);
	while(encoder0.get_cm() < 25) wait_ms(50);
	car.stop();

	// 24 : Reverse right
	step = 24;
	encoder0.reset();
	car.setLeftSpeed(-50);
	while(encoder0.get_steps() < 28) wait_ms(50);
	car.stop();
	
	// 25 : Move to exit Mission 2
	step = 25;
	encoder0.reset();
	car.goStraight(50);
	while(encoder0.get_cm() < 11) wait_ms(50);
	do {
		pval = getPing();
		wait(0.1);
	} while (pval > 23);
	car.stop();

	// 26 : Turn right to exit
	step = 26;
	encoder0.reset();
	car.setLeftSpeed(50);
	while(encoder0.get_steps() < 26) wait_ms(50);
	car.stop();

	// 27 : Straight to exit
	step = 27;
	encoder0.reset();
	car.goStraight(65);
	while(encoder0.get_cm() < 145) wait_ms(50);
	car.stop();

	// 28 : Finished
	step = 28;
	gLED = 1; bLED = 0;
}

float getPing() {
	float val;

	ping.output();
	ping = 0;
	wait_us(200);
	ping = 1;
	wait_us(5);
	ping = 0;
	wait_us(5);

	ping.input();
	while (ping.read() == 0);
	t.start();
	while (ping.read() == 1);
	val = t.read();
	t.stop();
	t.reset();

	return (val * 17700.4f);
}

void xbee_rx_interrupt(void)
{
	xbee.attach(NULL, Serial::RxIrq); // detach interrupt
	queue_xbee.call(&xbee_rx);
}

void xbee_rx(void)
{
	char buf[100] = {0};
	char outbuf[100] = {0};
	while (xbee.readable())
	{
		for (int i = 0;; i++)
		{
			char recv = xbee.getc();
			if (recv == '\r')
			{
				break;
			}
			buf[i] = pc.putc(recv);
		}
		
		pc.printf("%s\r\n", outbuf);
		wait(0.1);
	}
	xbee.attach(xbee_rx_interrupt, Serial::RxIrq); // reattach interrupt
}

void reply_messange(char *xbee_reply, char *messange)
{
	xbee_reply[0] = xbee.getc();
	xbee_reply[1] = xbee.getc();
	xbee_reply[2] = xbee.getc();
	if (xbee_reply[1] == 'O' && xbee_reply[2] == 'K')
	{
		pc.printf("%s\r\n", messange);
		xbee_reply[0] = '\0';
		xbee_reply[1] = '\0';
		xbee_reply[2] = '\0';
	}
}

void check_addr(char *xbee_reply, char *messenger)
{
	xbee_reply[0] = xbee.getc();
	xbee_reply[1] = xbee.getc();
	xbee_reply[2] = xbee.getc();
	xbee_reply[3] = xbee.getc();
	pc.printf("%s = %c%c%c\r\n", messenger, xbee_reply[1], xbee_reply[2], xbee_reply[3]);
	xbee_reply[0] = '\0';
	xbee_reply[1] = '\0';
	xbee_reply[2] = '\0';
	xbee_reply[3] = '\0';
}