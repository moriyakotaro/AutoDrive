#include "Sonoterm_forArduino.h"
Sonoterm* Sonoterm::pass = 0;

Sonoterm::Sonoterm(int _bps){
	bps = _bps;
	Serial.begin(_bps);
	pass = this;
	/*
	SCI = new UnbufferedSerial(tx,rx);
	SCI->format(8,SerialBase::None,1);
    SCI->baud(bps);
    SCI->attach(callback(this,&Sonoterm::receiveInt),UnbufferedSerial::RxIrq);*/
}
void serialEvent(){
	Sonoterm::funcPass();
}

void Sonoterm::funcPass(){
	pass->receiveInt();
}

void Sonoterm::init(){
	//Serial.begin(bps);
}

void Sonoterm::txd(char D){
    Serial.write(D);
}

void Sonoterm::txvalue(unsigned char D){
   if(D>=10)txd(0x3F);
	D = D+48;
    txd(D);
}

void Sonoterm::dnl(){
    txd(0x20);
    txd(0x0D);
	txd(0x0A);
}

void Sonoterm::putstr(char *TXDATA){
    while(*TXDATA != '\0'){
	    txd(*TXDATA);
	    TXDATA++;
    }
}

void Sonoterm::disp(long int D){
    unsigned char TD[5];
    int X=4;
    txd(0x20);
    if(D<0)txd(0x2D);
    else txd(0x2B);
    D = abs(D);
    while(X+1){
   	    TD[X] = D%10;
    	D /= 10;
    	X--;
	}
    for(X=0;X<5;X++)txvalue(TD[X]);
}


void  Sonoterm::comma(){
    txd(',');
}

int Sonoterm::advancedDisp(double opr, int pre){
    unsigned char digitNum[15], i;
    double scanningDigit = 10000.0;
    if(100000 < opr || opr < -100000 || 15 < pre)return -1;
    txd(0x20);
    if(opr < 0){
    	txd(0x2D);
    	opr *= -1;
    }
    while(((opr/scanningDigit)<1) && (scanningDigit != 1))scanningDigit /= 10.0;
    for(i=0;i<pre;i++){
    	if(scanningDigit == 0.1)txd('.');
    	digitNum[i] = opr/scanningDigit;
    	opr -= digitNum[i]*scanningDigit;
    	txvalue(digitNum[i]);
    	scanningDigit /= 10.0;
    }
    while(1 <= scanningDigit){
    	txvalue(0);
    	scanningDigit /= 10.0;
    }
    return 0;
}

void Sonoterm::DebugToW(double *motor1 ,double *motor2){/*
	if(RangeFlood(*motor1))return;
	if(RangeFlood(*motor2))return;*/
	if(debug_flg){
		if(start_sigh_flg){
			txd(0xA0);
			start_sigh_flg = false;
		}
		txd(0);
		FractionTx(*motor1);
		FractionTx(*motor2);
	}
	
	gain_addres[0][0][0] = motor1;
	gain_addres[0][0][1] = motor2;
}

void Sonoterm::DebugThW(double *motor1 ,double *motor2 ,double *motor3){
	/*
	if(RangeFlood(*motor1))return;
	if(RangeFlood(*motor2))return;
	if(RangeFlood(*motor3))return;
	*/
	if(debug_flg){
		if(start_sigh_flg){
			txd(0xA0);
			start_sigh_flg = false;
		}
		txd(10);
		FractionTx(*motor1);
		FractionTx(*motor2);
		FractionTx(*motor3);
	}
	
	gain_addres[1][0][0] = motor1;
	gain_addres[1][0][1] = motor2;
	gain_addres[1][0][2] = motor3;
}

void Sonoterm::DebugFW(double *motor1 ,double *motor2 ,double *motor3 ,double *motor4){
	/*
	if(RangeFlood(*motor1))return;
	if(RangeFlood(*motor2))return;
	if(RangeFlood(*motor3))return;
	if(RangeFlood(*motor4))return;
	*/
	if(debug_flg){
		if(start_sigh_flg){
			txd(0xA0);
			start_sigh_flg = false;
		}
		txd(20);
		FractionTx(*motor1);
		FractionTx(*motor2);
		FractionTx(*motor3);
		FractionTx(*motor4);
	}
	
	gain_addres[2][0][0] = motor1;
	gain_addres[2][0][1] = motor2;
	gain_addres[2][0][2] = motor3;
	gain_addres[2][0][3] = motor4;
}

void Sonoterm::DebugPID(double *Pgain ,double *Igain ,double *Dgain){
	
	if(debug_call_count[3] > 2){
		debug_call_count[3] = 183;
		return;
	}/*
	if(RangeFlood(*Pgain))return;
	if(RangeFlood(*Igain))return;
	if(RangeFlood(*Dgain))return;
	*/
	if(debug_flg){
		if(start_sigh_flg){
			txd(0xA0);
			start_sigh_flg = false;
		}
		txd(30 + debug_call_count[3]);
		FractionTx(*Pgain);
		FractionTx(*Igain);
		FractionTx(*Dgain);
	}
	
	gain_addres[3][debug_call_count[3]][0] = Pgain;
	gain_addres[3][debug_call_count[3]][1] = Igain;
	gain_addres[3][debug_call_count[3]][2] = Dgain;
	debug_call_count[3]++;
}

void Sonoterm::DebugEnd(){
	int i = 0;
	static bool end = false;
	if(end)txd(0xED);
	if(!(start_sigh_flg)){
		//debug_request_flg = false;
		debug_flg = false;
		end = false;
	}
	if(debug_request_flg){
		debug_request_flg = false;
		start_sigh_flg = true;
		debug_flg = true;
		end = true;
	}
	for(i = 0;i < 4;i++){
		debug_call_count[i] = 0;
	}
}

int Sonoterm::RangeFlood(double value){
	//if(value < 0) return 0;
	value = fabs(value);
	if((int)value > INTLIMIT) return 1;
	value = (value - (int)value) * 100000;
	if(value > DECLIMIT) return 1;
	return 0;
}

void Sonoterm::FractionTx(double value){
	//while(!(SCI.readable()));
	uint64_t int64;
	double indouble;
	double dc;
	memcpy(&int64,&value,sizeof(value));
	int64 = byteSwap(int64);
	memcpy(&indouble,&int64,sizeof(int64));
	dc = indouble;
    for(int i=0; i<sizeof(dc); i++){
		txd(((uint8_t*)(&dc))[i]);
	}
	/*
	if(value < 0){
		txd(0x2D);
		value *= -1;
	}else txd(0x2E);
	txd((int)value);
	int val = (value - (int)value) * 100000;
	txd(val >> 8);
	txd(val & 0xff);*/
	return;
}

uint64_t Sonoterm::byteSwap(uint64_t val){
    uint64_t ret;
    ret  =  val					  << 56;
    ret |= (val&0x000000000000FF00) << 40;
    ret |= (val&0x0000000000FF0000) << 24;
    ret |= (val&0x00000000FF000000) <<  8;
    ret |= (val&0x000000FF00000000) >>  8;
    ret |= (val&0x0000FF0000000000) >> 24;
    ret |= (val&0x00FF000000000000) >> 40;
    ret |=	val					  >> 56;
	return ret;
}

void Sonoterm::getGain(char *G){
	//static bool firstgain = true;
   		uint64_t int64;
		double indouble;
		double dc;
	if(G[0] == 0xA0){
	//char n[] = {64, -100, -92, -69, -108, 83, 8, -68};
   memcpy(&int64, &G[4], sizeof(double));
   int64 = byteSwap(int64);
   //int64 = bswap_64(int64);
   memcpy(&indouble, &int64, sizeof(int64));
   dc = indouble;
   //printf("%lf",dc);
		 *gain_addres[(int)G[1]][(int)G[2]][(int)G[3]] = dc;
	}
	debug_request_flg = true;
}

void Sonoterm::getKey(char *keyStat){
	//keys = keyStat[5];
	key_q.press 	= is(keyStat[2]&0x01);
	key_a.press 	= is(keyStat[2]&0x02);
	key_z.press 	= is(keyStat[2]&0x04);
	key_w.press 	= is(keyStat[2]&0x08);
	key_s.press 	= is(keyStat[2]&0x10);
	key_x.press 	= is(keyStat[2]&0x20);
	key_e.press 	= is(keyStat[2]&0x40);
	key_d.press 	= is(keyStat[3]&0x01);
	key_c.press 	= is(keyStat[3]&0x02);
	key_r.press 	= is(keyStat[3]&0x04);
	key_f.press 	= is(keyStat[3]&0x08);
	key_v.press 	= is(keyStat[3]&0x10);
	key_ta.press 	= is(keyStat[3]&0x20);
	key_g.press 	= is(keyStat[3]&0x40);
	key_b.press 	= is(keyStat[4]&0x01);
	key_y.press 	= is(keyStat[4]&0x02);
	key_h.press 	= is(keyStat[4]&0x04);
	key_n.press 	= is(keyStat[4]&0x08);
	key_u.press 	= is(keyStat[4]&0x10);
	key_j.press 	= is(keyStat[4]&0x20);
	key_m.press 	= is(keyStat[4]&0x40);
	key_i.press 	= is(keyStat[5]&0x01);
	key_k.press 	= is(keyStat[5]&0x02);
	key_o.press 	= is(keyStat[5]&0x04);
	key_l.press 	= is(keyStat[5]&0x08);
	key_p.press 	= is(keyStat[5]&0x10);
	shift.press 	= is(keyStat[5]&0x20);
	control.press 	= is(keyStat[5]&0x40);
	alt.press 	= is(keyStat[6]&0x01);
	backspace.press = is(keyStat[6]&0x02);
	tab.press 	= is(keyStat[6]&0x04);
	enter.press 	= is(keyStat[6]&0x08);
	esc.press 	= is(keyStat[6]&0x10);
	del.press 	= is(keyStat[6]&0x20);
	Up.press 	= is(keyStat[6]&0x40);
	Down.press 	= is(keyStat[7]&0x01);
	Right.press 	= is(keyStat[7]&0x02);
	Left.press 	= is(keyStat[7]&0x04);
	one.press 	= is(keyStat[7]&0x08);
	two.press 	= is(keyStat[7]&0x10);
	three.press 	= is(keyStat[7]&0x20);
	four.press 	= is(keyStat[7]&0x40);
	five.press 	= is(keyStat[8]&0x01);
	six.press 	= is(keyStat[8]&0x02);
	seven.press 	= is(keyStat[8]&0x04);
	eight.press 	= is(keyStat[8]&0x08);
	nine.press 	= is(keyStat[8]&0x10);
	zero.press 	= is(keyStat[8]&0x20);
	
	key_q.count 	+= pushCount(&key_q);
	key_a.count 	+= pushCount(&key_a);
	key_z.count 	+= pushCount(&key_z);
	key_w.count 	+= pushCount(&key_w);
	key_s.count 	+= pushCount(&key_s);
	key_x.count 	+= pushCount(&key_x);
	key_e.count 	+= pushCount(&key_e);
	key_d.count 	+= pushCount(&key_d);
	key_c.count 	+= pushCount(&key_c);
	key_r.count 	+= pushCount(&key_r);
	key_f.count 	+= pushCount(&key_f);
	key_v.count 	+= pushCount(&key_v);
	key_ta.count 	+= pushCount(&key_ta);
	key_g.count 	+= pushCount(&key_g);
	key_b.count 	+= pushCount(&key_b);
	key_y.count 	+= pushCount(&key_y);
	key_h.count 	+= pushCount(&key_h);
	key_n.count 	+= pushCount(&key_n);
	key_u.count 	+= pushCount(&key_u);
	key_j.count 	+= pushCount(&key_i);
	key_m.count 	+= pushCount(&key_m);
	key_i.count 	+= pushCount(&key_j);
	key_k.count 	+= pushCount(&key_k);
	key_o.count 	+= pushCount(&key_o);
	key_l.count 	+= pushCount(&key_l);
    key_p.count 	+= pushCount(&key_p);
    shift.count 	+= pushCount(&shift);
    control.count 	+= pushCount(&control);
    alt.count 	+= pushCount(&alt);
    backspace.count += pushCount(&backspace);
    tab.count 	+= pushCount(&tab);
    enter.count 	+= pushCount(&enter);
    esc.count 	+= pushCount(&esc);
    del.count 	+= pushCount(&del);
    Up.count 	+= pushCount(&Up);
    Down.count 	+= pushCount(&Down);
    Right.count 	+= pushCount(&Right);
    Left.count 	+= pushCount(&Left);
    one.count 	+= pushCount(&one);
    two.count 	+= pushCount(&two);
    three.count 	+= pushCount(&three);
    four.count 	+= pushCount(&four);
    five.count 	+= pushCount(&five);
    six.count 	+= pushCount(&six);
    seven.count 	+= pushCount(&seven);
    eight.count 	+= pushCount(&eight);
    nine.count 	+= pushCount(&nine);
    zero.count 	+= pushCount(&zero);
	
	key_q.toggle 	 = key_q.count%2;
	key_a.toggle 	 = key_a.count%2;
	key_z.toggle 	 = key_z.count%2;
	key_w.toggle 	 = key_w.count%2;
	key_s.toggle 	 = key_s.count%2;
	key_x.toggle 	 = key_x.count%2;
	key_e.toggle 	 = key_e.count%2;
	key_d.toggle 	 = key_d.count%2;
	key_c.toggle 	 = key_c.count%2;
	key_r.toggle 	 = key_r.count%2;
	key_f.toggle 	 = key_f.count%2;
	key_v.toggle 	 = key_v.count%2;
	key_ta.toggle 	 = key_ta.count%2;
	key_g.toggle 	 = key_g.count%2;
	key_b.toggle 	 = key_b.count%2;
	key_y.toggle 	 = key_y.count%2;
	key_h.toggle 	 = key_h.count%2;
	key_n.toggle 	 = key_n.count%2;
	key_u.toggle 	 = key_u.count%2;
	key_j.toggle 	 = key_j.count%2;
	key_m.toggle 	 = key_m.count%2;
	key_i.toggle 	 = key_i.count%2;
	key_k.toggle 	 = key_k.count%2;
	key_o.toggle 	 = key_o.count%2;
	key_l.toggle 	 = key_l.count%2;
	key_p.toggle 	 = key_p.count%2;
	shift.toggle 	 = shift.count%2;
	control.toggle 	 = control.count%2;
	alt.toggle 	 = alt.count%2;
	backspace.toggle = backspace.count%2; 
	tab.toggle 	 = tab.count%2;
	enter.toggle 	 = enter.count%2;
	esc.toggle 	 = esc.count%2;
	del.toggle 	 = del.count%2;
	Up.toggle 	 = Up.count%2;
	Down.toggle 	 = Down.count%2;
	Right.toggle 	 = Right.count%2;
	Left.toggle 	 = Left.count%2;
	one.toggle 	 = one.count%2;
	two.toggle 	 = two.count%2;
	three.toggle 	 = three.count%2;
	four.toggle 	 = four.count%2;
	five.toggle 	 = five.count%2;
	six.toggle 	 = six.count%2;
	seven.toggle 	 = seven.count%2;
	eight.toggle 	 = eight.count%2;
	nine.toggle 	 = nine.count%2;
	zero.toggle 	 = zero.count%2;
}

int Sonoterm::pushCount(Button *any){
	if(any->press){
		if(any->flg == 0){
			any->flg = 1;
			return 1;
		}
	}else{
		any->flg = 0;
	}
	return 0;
}

int Sonoterm::is(int num){
	return !!num;
}

void Sonoterm::receiveInt(){
	int i;
	int m = 0;
	static short DGsw = 0;
	static short key_data_num = 1830;
	//static char rx_buff;
	static int data_num = 0;
	static char rx_data[15];
	static char receive[15];

	rx_buff = Serial.read();
	if(!(data_num)){
		if(rx_buff == 0x80){
			DGsw = 1;
		}else
		if(rx_buff == 0xA0 || rx_buff == 0xA1){
			DGsw = 2;
		}
		for(i=0;i<15;i++){
			receive[i] = 0;
			rx_data[i] = 0;
		}
	}
	if(DGsw == 1){
		receive[data_num] = rx_buff;
		if(data_num == 1)key_data_num = bitCount(receive[data_num]);
		if(data_num  > key_data_num){
			for(i=0;i<7;i++)if(receive[1] & 1 << i){
				rx_data[i+2] = receive[2+m++];
			}
			data_num = -1;
			key_data_num = 1830;
			getKey(rx_data);
		}
		data_num++;
	}
	else if(DGsw == 2){
		receive[data_num++] = rx_buff;
		if(data_num >= 12){
			for(i=0; i<12; i++){
				rx_data[i] = receive[i];
			}
			data_num = 0;
			getGain(rx_data);
        }
	}
}

int Sonoterm::bitCount(int i){
	i = i - ((i >> 1) & 0x55);
    i = (i & 0x33) + ((i >> 2) & 0x33);
    i = (i + (i >> 4)) & 0x0f;
    return i & 0xf;
}
