#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/device.h>
#include <linux/input.h>
#define SLIDER_DRV_VER "231013"
// 230919:1.双指bug
//		  2.F13、F14上报
// 230922:1.双指左右分开
//		  2.tp 手掌抑制上报
//		  3.bug:滚动只报第一个动作
// 230927:1.键值修改成F13-F24
//        2.长按1S
// 231008:1.slider 只上报上滑、下滑、不动、双指，4种状态和触摸（DOWN)
// 231011:1.slider 滑动区分三种速度：间距分别为20、8、2
// 231013:1.slider 上报坐标,只报触摸和双指 两个key

#define RATTA_MT_DEBUG		0
#define RATTA_MT_NAME		"ratta-slide"
#define RATTA_MAX_FINGERS	5
#define RATTA_MAX_REC	100

#define RATTA_INVALID_VALUE	0x7FFFFFFF

#define RATTA_SLIDE_LEFT	30
#define RATTA_SLIDE_RIGHT	70
#define RATTA_SLIDE_OFFSET	120

#define KEY_STATUS_LEFT     0x01 //左边滑条
#define KEY_STATUS_RIGHT    0x02 //右边滑条

//#define KEY_STATUS_SHORT    0x04 //短按
//#define KEY_STATUS_LONG     0x08 //长按

#define KEY_STATUS_SLIDER_UP       0x04 //上滑
#define KEY_STATUS_SLIDER_DOWN     0x08 //下滑
#define KEY_STATUS_SLIDER_UP_1     0x10 //上滑
#define KEY_STATUS_SLIDER_DOWN_1   0x20 //下滑
#define KEY_STATUS_SLIDER_UP_2     0x40 //上滑
#define KEY_STATUS_SLIDER_DOWN_2   0x80 //下滑

//#define KEY_STATUS_ROLL_UP 	0x40 //上滚
//#define KEY_STATUS_ROLL_DOWN 0x80 //下滚

#define KEY_STATUS_UP       0x100 //弹起
#define KEY_STATUS_DOWN     0x200 //按下

#define KEY_STATUS_STAY     0x400 //还未滚动
#define KEY_STATUS_STOP     0x800 //停止

//#define KEY_STATUS_WAIT     0x1000 //等待滚动

#define KEY_STATUS_TWO_DOWN     0x2000 //双指按下
#define KEY_STATUS_TWO_UP     0x4000 //双指弹起

enum slider_tch_abs {	/* for ordering within the extracted touch data array */
	SLIDER_TCH_X,	/* X */
	SLIDER_TCH_Y,	/* Y */
	SLIDER_TCH_P,	/* P (Z) */
	SLIDER_TCH_T,	/* TOUCH ID */
	SLIDER_TCH_E,	/* EVENT ID */
	SLIDER_TCH_O,	/* OBJECT ID */
	SLIDER_TCH_TIP,	/* OBJECT ID */
	SLIDER_TCH_MAJ,	/* TOUCH_MAJOR */
	SLIDER_TCH_MIN,	/* TOUCH_MINOR */
	SLIDER_TCH_OR,	/* ORIENTATION */
	SLIDER_TCH_NUM_ABS,
};

enum finger_status {
	FINGER_NONE,
	FINGER_DOWN,
	FINGER_REPEAT,
	FINGER_SLIDER_UP,
	FINGER_SLIDER_UP_1,
	FINGER_SLIDER_UP_2,
	FINGER_SLIDER_DOWN,
	FINGER_SLIDER_DOWN_1,
	FINGER_SLIDER_DOWN_2,
	FINGER_TWO_PRESS,
	FINGER_UP,
	FINGER_DROP,
};

struct ratta_mt_data {
	/* start x and y position */
	int x, y, t, e, o, tip, num;
	unsigned long time;
	int done;
	int status;
};

struct ratta_mt_device {
	struct input_dev *input;
	struct ratta_mt_data data_record[2][RATTA_MAX_FINGERS][RATTA_MAX_REC];
	int rec_now[2][RATTA_MAX_FINGERS];
	int slider_left_count;
	int slider_right_count;
};
#if 0
enum slider_keys {
	SLIDER_L_UP,
	SLIDER_L_DOWN,
	SLIDER_L_ROLL_UP,
	SLIDER_L_ROLL_DOWN,
	SLIDER_L_TWO,
	SLIDER_L_LONG,
	SLIDER_R_UP,
	SLIDER_R_DOWN,
	SLIDER_R_ROLL_UP,
	SLIDER_R_ROLL_DOWN,
	SLIDER_R_TWO,
	SLIDER_R_LONG,
};
#endif
//F13--不动，F14--上滚      F15--上滚1 F16--上滚2 F17--下滚 F18--下滚1 F19--下滚2 F20--两指 F21--有触摸
enum slider_keys {
	SLIDER_L_STAY,
	SLIDER_L_ROLL_UP,
	SLIDER_L_ROLL_UP_1,
	SLIDER_L_ROLL_UP_2,
	SLIDER_L_ROLL_DOWN,
	SLIDER_L_ROLL_DOWN_1,
	SLIDER_L_ROLL_DOWN_2,
	SLIDER_L_TWO,
	SLIDER_L_DOWN,
	SLIDER_R_STAY,
	SLIDER_R_ROLL_UP,
	SLIDER_R_ROLL_UP_1,
	SLIDER_R_ROLL_UP_2,
	SLIDER_R_ROLL_DOWN,
	SLIDER_R_ROLL_DOWN_1,
	SLIDER_R_ROLL_DOWN_2,
	SLIDER_R_TWO,
	SLIDER_R_DOWN,
};

static struct ratta_mt_device *ratta_device = NULL;
int slider_mask;
unsigned int key_now = 0;
int key_map[]={KEY_F13,KEY_F14,KEY_F15,KEY_F16,KEY_F17,KEY_F18,KEY_F19,KEY_F20,KEY_F21,KEY_F22,KEY_F23,KEY_F24,KEY_F25,KEY_F26,KEY_F27,KEY_F28,KEY_F29,KEY_F30};
#define ratta_mt_debug(fmt, args...) do { \
		if (RATTA_MT_DEBUG) { \
			printk("[ratta_mt]%s:"fmt"\n", __func__, ##args); \
		} \
	} while (0);

static void ratta_report_slide(int code)
{
    printk("%s: keycode=%d\n", __func__, code);
	input_report_key(ratta_device->input,
			 code, 1);
	input_report_key(ratta_device->input,
			 code, 0);
	input_sync(ratta_device->input);
}

static void ratta_report_slide_updown(int code,int down)
{
	int i;
	if((code == SLIDER_L_DOWN)||(code == SLIDER_L_TWO)||(code == SLIDER_R_DOWN)||(code == SLIDER_R_TWO)){
		input_report_key(ratta_device->input,
				 code, down);
		input_sync(ratta_device->input);
		for(i=0;i<SLIDER_R_DOWN+1;i++){
			if(code == key_map[i]){
				break;
			}
		}
		if(down){
			key_now |= (1<<i);
		}else{
			key_now &= (~(1<<i));
		}
		printk("%s: keycode=%d down=%d key_now:0x%x i=%d\n", __func__, code,down,key_now,i);
	}else{
		printk("%s: keycode=%d down=%d no_function\n", __func__, code,down);
	}
}

static void ratta_slide_clean_keys(int mask)
{
	int i;

	if(key_now == 0){
		return;
	}
	if(mask & 0x01){
		for(i=0;i<SLIDER_R_STAY;i++){
			if(key_now & (1<<i)){
				input_report_key(ratta_device->input,
				 key_map[i], 0);
				printk("%s: keyup=%d key_now:0x%x\n", __func__, key_map[i], key_now);
			}
		}
		key_now &= 0x0FC0;
	}
	if(mask & 0x02){
		for(i=SLIDER_R_STAY;i<SLIDER_R_DOWN+1;i++){
			if(key_now & (1<<i)){
				input_report_key(ratta_device->input,
				 key_map[i], 0);
				printk("%s: keyup=%d key_now:0x%x\n", __func__, key_map[i], key_now);
			}
		}
		key_now &= 0x003F;
	}
	input_sync(ratta_device->input);
	
}

static void ratta_slide_clean_keys_except(int mask,int exceptkey)
{
	int i;

	if((key_now == 0) || (key_now == exceptkey)){
		return;
	}
	if(mask & 0x01){
		for(i=0;i<SLIDER_R_STAY;i++){
			if(key_now & (1<<i)){
				if((1<<i) & exceptkey){
				}else{
					input_report_key(ratta_device->input,
					 key_map[i], 0);
					key_now &= (~(1<<i));
					input_sync(ratta_device->input);
					printk("%s: keyup=%d key_now:0x%x\n", __func__, key_map[i],key_now);
				}
			}
		}
	}
	if(mask & 0x02){
		for(i=SLIDER_R_STAY;i<SLIDER_R_DOWN+1;i++){
			if(key_now & (1<<i)){
				if((1<<i) & exceptkey){
				}else{
					input_report_key(ratta_device->input,
					 key_map[i], 0);
					key_now &= (~(1<<i));
					input_sync(ratta_device->input);
					printk("%s: keyup=%d key_now:0x%x\n", __func__, key_map[i],key_now);
				}
			}
		}
	}
	//input_sync(ratta_device->input);
	//printk("%s: exceptkey=0x%x key_now:0x%x\n", __func__, exceptkey,key_now);

}

static void ratta_mt_clean(int mask)
{
	int record_num,i,j;
	for (i = 0; i < RATTA_MAX_FINGERS; i++) {	
		if((mask&0x01)&&(ratta_device->data_record[0][i][0].y == RATTA_SLIDE_LEFT)||
			(mask&0x02)&&(ratta_device->data_record[0][i][0].y == RATTA_SLIDE_RIGHT)){
			for (j = 0; j < RATTA_MAX_REC; j++) {
				ratta_device->data_record[0][i][j].x = RATTA_INVALID_VALUE;
				ratta_device->data_record[0][i][j].y = RATTA_INVALID_VALUE;
				ratta_device->data_record[0][i][j].t = RATTA_INVALID_VALUE;
				ratta_device->data_record[0][i][j].e = RATTA_INVALID_VALUE;
				ratta_device->data_record[0][i][j].o = RATTA_INVALID_VALUE;
				ratta_device->data_record[0][i][j].tip = RATTA_INVALID_VALUE;
				ratta_device->data_record[0][i][j].num = RATTA_INVALID_VALUE;
				ratta_device->data_record[0][i][j].time = 0;
				ratta_device->data_record[0][i][j].done = 0;
			}
			ratta_device->rec_now[0][i] = 0;
		}
	}
	if(mask&0x01)
		ratta_device->slider_left_count = 0;
	if(mask&0x02)
		ratta_device->slider_right_count = 0;
}


static void ratta_mt_dump(void)
{
	int record_num,i,j,type;
	for(type = 0; type < 2; type++) {
		for (i = 0; i < RATTA_MAX_FINGERS; i++) {
			for (j = 0; j < ratta_device->rec_now[type][i]; j++) {
				ratta_mt_debug("====ratta_mt_record[%d][%d][%d]= x:%d,y:%d,t:%d,e:%d,o:%d,tip:%d,num:%d,time:%ld \n",
					type,i,j,ratta_device->data_record[type][i][j].x,
					ratta_device->data_record[type][i][j].y,
					ratta_device->data_record[type][i][j].t,
					ratta_device->data_record[type][i][j].e,
					ratta_device->data_record[type][i][j].o,
					ratta_device->data_record[type][i][j].tip,
					ratta_device->data_record[type][i][j].num,
					ratta_device->data_record[type][i][j].time);
					//ratta_device->data_record[type][i][j].done
			}
		}
	}

}

static void ratta_mt_rerecord_finger(int track)
{
	int record_num,i,j;

	//if(ratta_device->data_record[0][track][0].status&(KEY_STATUS_ROLL_UP|KEY_STATUS_ROLL_DOWN|KEY_STATUS_STAY|KEY_STATUS_STOP|KEY_STATUS_WAIT|KEY_STATUS_TWO_DOWN))
		ratta_device->data_record[0][track][0].done = ratta_device->data_record[0][track][99].done;
	//else
	//	ratta_device->data_record[0][track][1].done = FINGER_DROP;
	ratta_device->data_record[0][track][0].x = ratta_device->data_record[0][track][99].x;
	ratta_device->data_record[0][track][0].y = ratta_device->data_record[0][track][99].y;
	ratta_device->data_record[0][track][0].t = ratta_device->data_record[0][track][99].t;
	ratta_device->data_record[0][track][0].e = ratta_device->data_record[0][track][99].e;
	ratta_device->data_record[0][track][0].o = ratta_device->data_record[0][track][99].o;
	ratta_device->data_record[0][track][0].tip = ratta_device->data_record[0][track][99].tip;
	ratta_device->data_record[0][track][0].num = ratta_device->data_record[0][track][99].num;
	ratta_device->data_record[0][track][0].time = ratta_device->data_record[0][track][99].time;

	ratta_mt_debug("====%s :track:%d,x:%d,y:%d,t:%d,tip:%d,time:%ld done:%d statue:%d\n",
		__func__,track,ratta_device->data_record[0][track][99].x,ratta_device->data_record[0][track][99].y,
		ratta_device->data_record[0][track][99].t,ratta_device->data_record[0][track][99].tip,
		ratta_device->data_record[0][track][99].time,ratta_device->data_record[0][track][1].done,
		ratta_device->data_record[0][track][i].status);


	for (i = 1; i < ratta_device->rec_now[0][track]; i++) {
		ratta_device->data_record[0][track][i].x = RATTA_INVALID_VALUE;
		ratta_device->data_record[0][track][i].y = RATTA_INVALID_VALUE;
		ratta_device->data_record[0][track][i].t = RATTA_INVALID_VALUE;
		ratta_device->data_record[0][track][i].e = RATTA_INVALID_VALUE;
		ratta_device->data_record[0][track][i].o = RATTA_INVALID_VALUE;
		ratta_device->data_record[0][track][i].tip = RATTA_INVALID_VALUE;
		ratta_device->data_record[0][track][i].num = RATTA_INVALID_VALUE;
		ratta_device->data_record[0][track][i].time = 0;
		ratta_device->data_record[0][track][i].done = 0;
		ratta_device->data_record[0][track][i].status = 0;
	}
	ratta_device->rec_now[0][track] = 1;
}

static void ratta_mt_clean_finger(int type,int track)
{
	int record_num,i,j;
	ratta_mt_debug("%s track:%d rec:%d \n",__func__,track,ratta_device->rec_now[type][track]);
	for (i = 0; i < ratta_device->rec_now[type][track]; i++) {
		ratta_device->data_record[type][track][i].x = RATTA_INVALID_VALUE;
		ratta_device->data_record[type][track][i].y = RATTA_INVALID_VALUE;
		ratta_device->data_record[type][track][i].t = RATTA_INVALID_VALUE;
		ratta_device->data_record[type][track][i].e = RATTA_INVALID_VALUE;
		ratta_device->data_record[type][track][i].o = RATTA_INVALID_VALUE;
		ratta_device->data_record[type][track][i].tip = RATTA_INVALID_VALUE;
		ratta_device->data_record[type][track][i].num = RATTA_INVALID_VALUE;
		ratta_device->data_record[type][track][i].time = 0;
		ratta_device->data_record[type][track][i].done = 0;
		ratta_device->data_record[type][track][i].status = 0;
	}
	ratta_device->rec_now[type][track] = 0;
}

static void ratta_mt_add_record(int tch[], unsigned long jiffs)
{
	int record_num;

	record_num = ratta_device->rec_now[0][tch[SLIDER_TCH_T]];
	ratta_device->data_record[0][tch[SLIDER_TCH_T]][record_num].x = tch[SLIDER_TCH_X];
	ratta_device->data_record[0][tch[SLIDER_TCH_T]][record_num].y = tch[SLIDER_TCH_Y];
	ratta_device->data_record[0][tch[SLIDER_TCH_T]][record_num].t = tch[SLIDER_TCH_T];
	ratta_device->data_record[0][tch[SLIDER_TCH_T]][record_num].e = tch[SLIDER_TCH_E];
	ratta_device->data_record[0][tch[SLIDER_TCH_T]][record_num].o = tch[SLIDER_TCH_O];
	ratta_device->data_record[0][tch[SLIDER_TCH_T]][record_num].tip = tch[SLIDER_TCH_TIP];
	ratta_device->data_record[0][tch[SLIDER_TCH_T]][record_num].num = tch[SLIDER_TCH_NUM_ABS];
	ratta_device->data_record[0][tch[SLIDER_TCH_T]][record_num].time = jiffs;
	ratta_device->rec_now[0][tch[SLIDER_TCH_T]]++;
	ratta_mt_debug("====ratta_mt_add_record rec:%d t:%d,x:%d,y:%d,t:%d,e:%d,o:%d,tip:%d,num:%d,time:%ld \n",
		record_num,tch[SLIDER_TCH_T],tch[SLIDER_TCH_X],tch[SLIDER_TCH_Y],tch[SLIDER_TCH_T],tch[SLIDER_TCH_E],
		tch[SLIDER_TCH_O],tch[SLIDER_TCH_TIP],tch[SLIDER_TCH_NUM_ABS],jiffs);

}
static void ratta_mt_parse(int track)
{
	int record_num,i,j;
	int slider_left = 0,slider_right = 0;
	int keycode = 0;
	int key_status = 0;
	int last = 0;
	int need_parse[2][RATTA_MAX_FINGERS];
	bool is_same=true;
	int time ,repeat_time;

	for (i = 0; i < RATTA_MAX_FINGERS; i++) {
		if(ratta_device->rec_now[0][i] >0){
			if(ratta_device->data_record[0][i][0].y == RATTA_SLIDE_LEFT){
				need_parse[0][slider_left]=i;
				slider_left++;
			}else if(ratta_device->data_record[0][i][0].y == RATTA_SLIDE_RIGHT){
				need_parse[1][slider_right]=i;
				slider_right++;
			}
		}
	}
	last = ratta_device->rec_now[0][track]-1;
	time = jiffies_to_msecs(ratta_device->data_record[0][track][last].time-ratta_device->data_record[0][track][0].time);
ratta_mt_debug("%s track:%d left:%d right:%d last:%d lasttime:%ld 0time:%ld time:%d rec:%d\n",__func__,
	track,slider_left,slider_right,last,ratta_device->data_record[0][track][last].time,ratta_device->data_record[0][track][0].time,time,ratta_device->rec_now[0][track]);
	if(ratta_device->data_record[0][track][0].y==RATTA_SLIDE_LEFT && ratta_device->data_record[0][track][last].y==RATTA_SLIDE_LEFT)
		key_status |= KEY_STATUS_LEFT;
	if(ratta_device->data_record[0][track][0].y==RATTA_SLIDE_RIGHT && ratta_device->data_record[0][track][last].y==RATTA_SLIDE_RIGHT)
		key_status |= KEY_STATUS_RIGHT;

	if((ratta_device->slider_left_count > 2)&&(ratta_device->data_record[0][track][0].y==RATTA_SLIDE_LEFT)){
		if(ratta_device->data_record[0][track][last].tip == 0){
			ratta_mt_clean_finger(0,track);
			return;//left area more then 2 fingers
		}
		return;
	}else if((ratta_device->slider_right_count > 2)&&(ratta_device->data_record[0][track][0].y==RATTA_SLIDE_RIGHT)){
		if(ratta_device->data_record[0][track][last].tip == 0){
			ratta_mt_clean_finger(0,track);
			return;//right area more then 2 fingers
		}
		return;
	}
	if((last>0) && (ratta_device->data_record[0][track][last-1].done==ratta_device->data_record[0][track][last].done)){
		return;
	}

	if((slider_left == 2)&&(ratta_device->data_record[0][track][0].y==RATTA_SLIDE_LEFT)){
		//wait left 2 fingers up at one time
		if((ratta_device->data_record[0][need_parse[0][0]][last].done == FINGER_DOWN) || (ratta_device->data_record[0][need_parse[0][1]][last].done == FINGER_DOWN)){
			ratta_slide_clean_keys_except(1,1<<SLIDER_L_DOWN);
			ratta_report_slide_updown(key_map[SLIDER_L_TWO],1);//重复按下会不会有问题？
			ratta_device->data_record[0][need_parse[0][0]][0].status = KEY_STATUS_TWO_DOWN;
			ratta_device->data_record[0][need_parse[0][1]][0].status = KEY_STATUS_TWO_DOWN;
		}else if((ratta_device->data_record[0][track][0].status == KEY_STATUS_TWO_DOWN)&&(ratta_device->data_record[0][track][last].tip == 0)){
			ratta_mt_clean_finger(0,track);
			ratta_slide_clean_keys_except(1,1<<SLIDER_L_DOWN);
			//ratta_report_slide_updown(key_map[SLIDER_L_TWO],0);
			ratta_device->data_record[0][need_parse[0][0]][0].status = 0;
			ratta_device->data_record[0][need_parse[0][1]][0].status = 0;
			if(need_parse[0][0] == track){
				ratta_device->data_record[0][need_parse[0][1]][last].done = FINGER_DROP;
			}else{
				ratta_device->data_record[0][need_parse[0][0]][last].done = FINGER_DROP;
			}
		}
	}else if((slider_right == 2)&&(ratta_device->data_record[0][track][0].y==RATTA_SLIDE_RIGHT)){
		//wait right 2 fingers up at one time
		if((ratta_device->data_record[0][need_parse[1][0]][last].done == FINGER_DOWN) || (ratta_device->data_record[0][need_parse[1][1]][last].done == FINGER_DOWN)){
			ratta_slide_clean_keys_except(2,1<<SLIDER_R_DOWN);
			ratta_report_slide_updown(key_map[SLIDER_R_TWO],1);//重复按下会不会有问题？
			ratta_device->data_record[0][need_parse[1][0]][0].status = KEY_STATUS_TWO_DOWN;
			ratta_device->data_record[0][need_parse[1][1]][0].status = KEY_STATUS_TWO_DOWN;
		}else if((ratta_device->data_record[0][track][0].status == KEY_STATUS_TWO_DOWN)&&(ratta_device->data_record[0][track][last].tip == 0)){
			ratta_mt_clean_finger(0,track);
			ratta_slide_clean_keys_except(2,1<<SLIDER_R_DOWN);
			//ratta_report_slide_updown(key_map[SLIDER_R_TWO],0);
			ratta_device->data_record[0][need_parse[1][0]][0].status = 0;
			ratta_device->data_record[0][need_parse[1][1]][0].status = 0;
			if(need_parse[1][0] == track){
				ratta_device->data_record[0][need_parse[1][1]][last].done = FINGER_DROP;
			}else{
				ratta_device->data_record[0][need_parse[1][0]][last].done = FINGER_DROP;
			}
		}
	}
	if((slider_left == 1)&&(key_status&KEY_STATUS_LEFT)){
		//report left key
		switch (ratta_device->data_record[0][track][last].done){
			case FINGER_NONE:
				break;
			case FINGER_DOWN:
				key_status |= KEY_STATUS_DOWN;
				break;
			case FINGER_REPEAT:
				key_status |= KEY_STATUS_STAY;
				break;
			case FINGER_SLIDER_UP:
				key_status |= KEY_STATUS_SLIDER_UP;
				break;
			case FINGER_SLIDER_UP_1:
				key_status |= KEY_STATUS_SLIDER_UP_1;
				break;
			case FINGER_SLIDER_UP_2:
				key_status |= KEY_STATUS_SLIDER_UP_2;
				break;
			case FINGER_SLIDER_DOWN:
				key_status |= KEY_STATUS_SLIDER_DOWN;
				break;
			case FINGER_SLIDER_DOWN_1:
				key_status |= KEY_STATUS_SLIDER_DOWN_1;
				break;
			case FINGER_SLIDER_DOWN_2:
				key_status |= KEY_STATUS_SLIDER_DOWN_2;
				break;
			case FINGER_TWO_PRESS:
				break;
			case FINGER_UP:
				key_status |= KEY_STATUS_UP;
				break;
			case FINGER_DROP:
				key_status |= KEY_STATUS_STOP;
				break;
		}
		if(key_status&KEY_STATUS_UP){
			ratta_device->data_record[0][track][0].status = 0;
			ratta_mt_clean_finger(0,track);
			ratta_slide_clean_keys(1);
			return ;
		}
		if(key_status&KEY_STATUS_STOP){
			ratta_device->data_record[0][track][0].status = KEY_STATUS_STOP;
			//ratta_report_slide_updown(key_map[SLIDER_L_DOWN],1);
			return ;
		}
		if(ratta_device->data_record[0][track][0].status == KEY_STATUS_STOP){
			//key_status |= KEY_STATUS_STOP;
			return ;
		}
		if(key_status&KEY_STATUS_DOWN){
			ratta_device->data_record[0][track][0].status |= KEY_STATUS_DOWN;
			ratta_report_slide_updown(key_map[SLIDER_L_DOWN],1);
			return ;
		}
		
		if(key_status&KEY_STATUS_STAY){
			ratta_device->data_record[0][track][0].status = (KEY_STATUS_DOWN|KEY_STATUS_STAY);
			ratta_slide_clean_keys_except(1,1<<SLIDER_L_DOWN);
			ratta_report_slide_updown(key_map[SLIDER_L_STAY],1);
			return ;
		}
		if(key_status&KEY_STATUS_SLIDER_UP){
			ratta_device->data_record[0][track][0].status = (KEY_STATUS_DOWN|KEY_STATUS_SLIDER_UP);
			ratta_slide_clean_keys_except(1,1<<SLIDER_L_DOWN);
			ratta_report_slide_updown(key_map[SLIDER_L_ROLL_UP],1);
			return ;
		}
		if(key_status&KEY_STATUS_SLIDER_UP_1){
			ratta_device->data_record[0][track][0].status = (KEY_STATUS_DOWN|KEY_STATUS_SLIDER_UP_1);
			ratta_slide_clean_keys_except(1,1<<SLIDER_L_DOWN);
			ratta_report_slide_updown(key_map[SLIDER_L_ROLL_UP_1],1);
			return ;
		}
		if(key_status&KEY_STATUS_SLIDER_UP_2){
			ratta_device->data_record[0][track][0].status = (KEY_STATUS_DOWN|KEY_STATUS_SLIDER_UP_2);
			ratta_slide_clean_keys_except(1,1<<SLIDER_L_DOWN);
			ratta_report_slide_updown(key_map[SLIDER_L_ROLL_UP_2],1);
			return ;
		}
		if(key_status&KEY_STATUS_SLIDER_DOWN){
			ratta_device->data_record[0][track][0].status = (KEY_STATUS_DOWN|KEY_STATUS_SLIDER_DOWN);
			ratta_slide_clean_keys_except(1,1<<SLIDER_L_DOWN);
			ratta_report_slide_updown(key_map[SLIDER_L_ROLL_DOWN],1);
			return ;
		}
		if(key_status&KEY_STATUS_SLIDER_DOWN_1){
			ratta_device->data_record[0][track][0].status = (KEY_STATUS_DOWN|KEY_STATUS_SLIDER_DOWN_1);
			ratta_slide_clean_keys_except(1,1<<SLIDER_L_DOWN);
			ratta_report_slide_updown(key_map[SLIDER_L_ROLL_DOWN_1],1);
			return ;
		}
		if(key_status&KEY_STATUS_SLIDER_DOWN_2){
			ratta_device->data_record[0][track][0].status = (KEY_STATUS_DOWN|KEY_STATUS_SLIDER_DOWN_2);
			ratta_slide_clean_keys_except(1,1<<SLIDER_L_DOWN);
			ratta_report_slide_updown(key_map[SLIDER_L_ROLL_DOWN_2],1);
			return ;
		}
	}else if((slider_right == 1)&&(key_status&KEY_STATUS_RIGHT)){
		//report right key
		switch (ratta_device->data_record[0][track][last].done){
			case FINGER_NONE:
				break;
			case FINGER_DOWN:
				key_status |= KEY_STATUS_DOWN;
				break;
			case FINGER_REPEAT:
				key_status |= KEY_STATUS_STAY;
				break;
			case FINGER_SLIDER_UP:
				key_status |= KEY_STATUS_SLIDER_UP;
				break;
			case FINGER_SLIDER_UP_1:
				key_status |= KEY_STATUS_SLIDER_UP_1;
				break;
			case FINGER_SLIDER_UP_2:
				key_status |= KEY_STATUS_SLIDER_UP_2;
				break;
			case FINGER_SLIDER_DOWN:
				key_status |= KEY_STATUS_SLIDER_DOWN;
				break;
			case FINGER_SLIDER_DOWN_1:
				key_status |= KEY_STATUS_SLIDER_DOWN_1;
				break;
			case FINGER_SLIDER_DOWN_2:
				key_status |= KEY_STATUS_SLIDER_DOWN_2;
				break;
			case FINGER_TWO_PRESS:
				break;
			case FINGER_UP:
				key_status |= KEY_STATUS_UP;
				break;
			case FINGER_DROP:
				key_status |= KEY_STATUS_STOP;
				break;
		}
		if(key_status&KEY_STATUS_UP){
			ratta_device->data_record[0][track][0].status = 0;
			ratta_mt_clean_finger(0,track);
			ratta_slide_clean_keys(2);
			return ;
		}
		if(key_status&KEY_STATUS_STOP){
			ratta_device->data_record[0][track][0].status = KEY_STATUS_STOP;
			//ratta_report_slide_updown(key_map[SLIDER_L_DOWN],1);
			return ;
		}
		if(ratta_device->data_record[0][track][0].status == KEY_STATUS_STOP){
			//key_status |= KEY_STATUS_STOP;
			return ;
		}
		if(key_status&KEY_STATUS_DOWN){
			ratta_device->data_record[0][track][0].status |= KEY_STATUS_DOWN;
			ratta_report_slide_updown(key_map[SLIDER_R_DOWN],1);
			return ;
		}
		
		if(key_status&KEY_STATUS_STAY){
			ratta_device->data_record[0][track][0].status = (KEY_STATUS_DOWN|KEY_STATUS_STAY);
			ratta_slide_clean_keys_except(2,1<<SLIDER_R_DOWN);
			ratta_report_slide_updown(key_map[SLIDER_R_STAY],1);
			return ;
		}
		if(key_status&KEY_STATUS_SLIDER_UP){
			ratta_device->data_record[0][track][0].status = (KEY_STATUS_DOWN|KEY_STATUS_SLIDER_UP);
			ratta_slide_clean_keys_except(2,1<<SLIDER_R_DOWN);
			ratta_report_slide_updown(key_map[SLIDER_R_ROLL_UP],1);
			return ;
		}
		if(key_status&KEY_STATUS_SLIDER_UP_1){
			ratta_device->data_record[0][track][0].status = (KEY_STATUS_DOWN|KEY_STATUS_SLIDER_UP_1);
			ratta_slide_clean_keys_except(2,1<<SLIDER_R_DOWN);
			ratta_report_slide_updown(key_map[SLIDER_R_ROLL_UP_1],1);
			return ;
		}
		if(key_status&KEY_STATUS_SLIDER_UP_2){
			ratta_device->data_record[0][track][0].status = (KEY_STATUS_DOWN|KEY_STATUS_SLIDER_UP_2);
			ratta_slide_clean_keys_except(2,1<<SLIDER_R_DOWN);
			ratta_report_slide_updown(key_map[SLIDER_R_ROLL_UP_2],1);
			return ;
		}
		if(key_status&KEY_STATUS_SLIDER_DOWN){
			ratta_device->data_record[0][track][0].status = (KEY_STATUS_DOWN|KEY_STATUS_SLIDER_DOWN);
			ratta_slide_clean_keys_except(2,1<<SLIDER_R_DOWN);
			ratta_report_slide_updown(key_map[SLIDER_R_ROLL_DOWN],1);
			return ;
		}
		if(key_status&KEY_STATUS_SLIDER_DOWN_1){
			ratta_device->data_record[0][track][0].status = (KEY_STATUS_DOWN|KEY_STATUS_SLIDER_DOWN_1);
			ratta_slide_clean_keys_except(2,1<<SLIDER_R_DOWN);
			ratta_report_slide_updown(key_map[SLIDER_R_ROLL_DOWN_1],1);
			return ;
		}
		if(key_status&KEY_STATUS_SLIDER_DOWN_2){
			ratta_device->data_record[0][track][0].status = (KEY_STATUS_DOWN|KEY_STATUS_SLIDER_DOWN_2);
			ratta_slide_clean_keys_except(2,1<<SLIDER_R_DOWN);
			ratta_report_slide_updown(key_map[SLIDER_R_ROLL_DOWN_2],1);
			return ;
		}
	}
}

/*
 * @id: track id
 * @x: x position
 * @y: y position
 * @state: true for touch and false for leave
 */
int ratta_mt_record(int type, bool record, int track, int tch[], unsigned long jiffs)
{
	//struct ratta_mt_record *pr;
    int i = 0;
    int record_num = 0;
	unsigned int time;
	if((slider_mask&0x01)&&(tch[SLIDER_TCH_Y] == RATTA_SLIDE_LEFT)){
		ratta_mt_clean(slider_mask);
		ratta_slide_clean_keys(slider_mask);
		return 0;
	}
	if((slider_mask&0x02)&&(tch[SLIDER_TCH_Y] == RATTA_SLIDE_RIGHT)){
		ratta_mt_clean(slider_mask);
		ratta_slide_clean_keys(slider_mask);
		return 0;
	}
	if (!ratta_device)
		return -ENODEV;
	//ratta_mt_debug
	ratta_mt_debug("====ratta_mt_record type:%d rec:%d t:%d,x:%d,y:%d,t:%d,e:%d,o:%d,tip:%d,num:%d,time:%ld \n",
		type,record,track,tch[SLIDER_TCH_X],tch[SLIDER_TCH_Y],tch[SLIDER_TCH_T],tch[SLIDER_TCH_E],
		tch[SLIDER_TCH_O],tch[SLIDER_TCH_TIP],tch[SLIDER_TCH_NUM_ABS],jiffs);
	if ((track < 0) || (track >= RATTA_MAX_FINGERS)) {
		return -EINVAL;
	}
	if(record == 0){
		ratta_mt_clean(3);
		return 0;
	}
//if(ratta_device->slider_left_count)
	record_num = ratta_device->rec_now[type][tch[SLIDER_TCH_T]];
	if(record_num > 99){
		ratta_mt_rerecord_finger(track);// todo:repeat 时间计算。
		record_num = 1;
		//return -EINVAL;
	}
	if(record_num!=0){
		time = jiffies_to_msecs(jiffs-ratta_device->data_record[0][track][record_num-1].time);
	}else{
		time = 0;
	}
	
	ratta_mt_debug("%s record_num:%d time:%d",__func__,record_num,time);
	//1.抬起------添加
	//2.不存在的手指---------添加
	//3.存在的手指  
	//	1.5ms，判断一次

	//1----------------------------------
	if(tch[SLIDER_TCH_TIP]==0){
		ratta_mt_add_record(tch, jiffs);
		ratta_device->data_record[0][tch[SLIDER_TCH_T]][record_num].done = FINGER_UP;
		goto parse;
	}
	//2-----------------------------------
	if(ratta_device->data_record[0][tch[SLIDER_TCH_T]][0].x==RATTA_INVALID_VALUE){
		ratta_mt_add_record(tch, jiffs);
		ratta_device->data_record[0][tch[SLIDER_TCH_T]][record_num].done = FINGER_DOWN;
		goto parse;
	}
	//3-----------------------------------
	//if(jiffies_to_msecs(jiffs-ratta_device->data_record[0][i][record_num-1].time)<RATTA_SLIDE_INTERVAL)
	//	return 0;
	if(ratta_device->data_record[0][i][record_num-1].y==tch[SLIDER_TCH_Y]){
		if(ratta_device->data_record[0][i][record_num-1].x==tch[SLIDER_TCH_X]){
			if(time > 100){
				ratta_mt_add_record(tch, jiffs);
				ratta_device->data_record[0][tch[SLIDER_TCH_T]][record_num].done = FINGER_REPEAT;
			}else{
				return 0;
			}
		}else if(ratta_device->data_record[0][i][record_num-1].x-tch[SLIDER_TCH_X]>2){
			//if((ratta_device->data_record[0][i][record_num-1].done == FINGER_SLIDER_DOWN)||
			//	(ratta_device->data_record[0][i][record_num-1].done == FINGER_DOWN)){
			ratta_mt_add_record(tch, jiffs);
			if((ratta_device->data_record[0][tch[SLIDER_TCH_T]][record_num-1].done!=FINGER_SLIDER_UP)&&
				(ratta_device->data_record[0][tch[SLIDER_TCH_T]][record_num-1].done!=FINGER_SLIDER_UP_1)&&
				(ratta_device->data_record[0][tch[SLIDER_TCH_T]][record_num-1].done!=FINGER_SLIDER_UP_2)){
				if(ratta_device->data_record[0][i][record_num-1].x-tch[SLIDER_TCH_X]>20){
					ratta_device->data_record[0][tch[SLIDER_TCH_T]][record_num].done = FINGER_SLIDER_UP_2;
				}else if(ratta_device->data_record[0][i][record_num-1].x-tch[SLIDER_TCH_X]>8){
					ratta_device->data_record[0][tch[SLIDER_TCH_T]][record_num].done = FINGER_SLIDER_UP_1;
				}else{
					ratta_device->data_record[0][tch[SLIDER_TCH_T]][record_num].done = FINGER_SLIDER_UP;
				}
			}else{
				if(ratta_device->data_record[0][i][record_num-1].x-tch[SLIDER_TCH_X]>22){
					ratta_device->data_record[0][tch[SLIDER_TCH_T]][record_num].done = FINGER_SLIDER_UP_2;
				}else if(ratta_device->data_record[0][i][record_num-1].x-tch[SLIDER_TCH_X]>20){
					if(ratta_device->data_record[0][tch[SLIDER_TCH_T]][record_num-1].done!=FINGER_SLIDER_UP_1){
						ratta_device->data_record[0][tch[SLIDER_TCH_T]][record_num].done = FINGER_SLIDER_UP_2;
					}else{
						ratta_device->data_record[0][tch[SLIDER_TCH_T]][record_num].done = FINGER_SLIDER_UP_1;
					}
				}else if(ratta_device->data_record[0][i][record_num-1].x-tch[SLIDER_TCH_X]>18){
					if(ratta_device->data_record[0][tch[SLIDER_TCH_T]][record_num-1].done==FINGER_SLIDER_UP_2){
						ratta_device->data_record[0][tch[SLIDER_TCH_T]][record_num].done = FINGER_SLIDER_UP_2;
					}else{
						ratta_device->data_record[0][tch[SLIDER_TCH_T]][record_num].done = FINGER_SLIDER_UP_1;
					}
				}else if(ratta_device->data_record[0][i][record_num-1].x-tch[SLIDER_TCH_X]>8){
					ratta_device->data_record[0][tch[SLIDER_TCH_T]][record_num].done = FINGER_SLIDER_UP_1;
				}else if(ratta_device->data_record[0][i][record_num-1].x-tch[SLIDER_TCH_X]>6){
					if(ratta_device->data_record[0][tch[SLIDER_TCH_T]][record_num-1].done!=FINGER_SLIDER_UP_1){
						ratta_device->data_record[0][tch[SLIDER_TCH_T]][record_num].done = FINGER_SLIDER_UP;
					}else{
						ratta_device->data_record[0][tch[SLIDER_TCH_T]][record_num].done = FINGER_SLIDER_UP_1;
					}
				}else{
					ratta_device->data_record[0][tch[SLIDER_TCH_T]][record_num].done = FINGER_SLIDER_UP;
				}
			}
			if(ratta_device->data_record[0][i][record_num-1].x-tch[SLIDER_TCH_X]>20){
				ratta_device->data_record[0][tch[SLIDER_TCH_T]][record_num].done = FINGER_SLIDER_UP_2;
			}else if(ratta_device->data_record[0][i][record_num-1].x-tch[SLIDER_TCH_X]>8){
				ratta_device->data_record[0][tch[SLIDER_TCH_T]][record_num].done = FINGER_SLIDER_UP_1;
			}else{
				ratta_device->data_record[0][tch[SLIDER_TCH_T]][record_num].done = FINGER_SLIDER_UP;
			}
			//}
		}else if(tch[SLIDER_TCH_X]-ratta_device->data_record[0][i][record_num-1].x>2){
			ratta_mt_add_record(tch, jiffs);
			//ratta_device->data_record[0][tch[SLIDER_TCH_T]][record_num].done = FINGER_SLIDER_DOWN;
			if((ratta_device->data_record[0][tch[SLIDER_TCH_T]][record_num-1].done!=FINGER_SLIDER_DOWN)&&
				(ratta_device->data_record[0][tch[SLIDER_TCH_T]][record_num-1].done!=FINGER_SLIDER_DOWN_1)&&
				(ratta_device->data_record[0][tch[SLIDER_TCH_T]][record_num-1].done!=FINGER_SLIDER_DOWN_2)){
				if(tch[SLIDER_TCH_X]-ratta_device->data_record[0][i][record_num-1].x>20){
					ratta_device->data_record[0][tch[SLIDER_TCH_T]][record_num].done = FINGER_SLIDER_DOWN_2;
				}else if(tch[SLIDER_TCH_X]-ratta_device->data_record[0][i][record_num-1].x>8){
					ratta_device->data_record[0][tch[SLIDER_TCH_T]][record_num].done = FINGER_SLIDER_DOWN_1;
				}else{
					ratta_device->data_record[0][tch[SLIDER_TCH_T]][record_num].done = FINGER_SLIDER_DOWN;
				}
			}else{
				if(tch[SLIDER_TCH_X]-ratta_device->data_record[0][i][record_num-1].x>22){
					ratta_device->data_record[0][tch[SLIDER_TCH_T]][record_num].done = FINGER_SLIDER_DOWN_2;
				}else if(tch[SLIDER_TCH_X]-ratta_device->data_record[0][i][record_num-1].x>20){
					if(ratta_device->data_record[0][tch[SLIDER_TCH_T]][record_num-1].done!=FINGER_SLIDER_DOWN_1){
						ratta_device->data_record[0][tch[SLIDER_TCH_T]][record_num].done = FINGER_SLIDER_DOWN_2;
					}else{
						ratta_device->data_record[0][tch[SLIDER_TCH_T]][record_num].done = FINGER_SLIDER_DOWN_1;
					}
				}else if(tch[SLIDER_TCH_X]-ratta_device->data_record[0][i][record_num-1].x>18){
					if(ratta_device->data_record[0][tch[SLIDER_TCH_T]][record_num-1].done==FINGER_SLIDER_DOWN_2){
						ratta_device->data_record[0][tch[SLIDER_TCH_T]][record_num].done = FINGER_SLIDER_DOWN_2;
					}else{
						ratta_device->data_record[0][tch[SLIDER_TCH_T]][record_num].done = FINGER_SLIDER_DOWN_1;
					}
				}else if(tch[SLIDER_TCH_X]-ratta_device->data_record[0][i][record_num-1].x>8){
					ratta_device->data_record[0][tch[SLIDER_TCH_T]][record_num].done = FINGER_SLIDER_DOWN_1;
				}else if(tch[SLIDER_TCH_X]-ratta_device->data_record[0][i][record_num-1].x>6){
					if(ratta_device->data_record[0][tch[SLIDER_TCH_T]][record_num-1].done!=FINGER_SLIDER_DOWN_1){
						ratta_device->data_record[0][tch[SLIDER_TCH_T]][record_num].done = FINGER_SLIDER_DOWN;
					}else{
						ratta_device->data_record[0][tch[SLIDER_TCH_T]][record_num].done = FINGER_SLIDER_DOWN_1;
					}
				}else{
					ratta_device->data_record[0][tch[SLIDER_TCH_T]][record_num].done = FINGER_SLIDER_DOWN;
				}
			}
		}else{
			if(time > 500){
				ratta_mt_add_record(tch, jiffs);
				ratta_device->data_record[0][tch[SLIDER_TCH_T]][record_num].done = FINGER_REPEAT;
			}else{
				return 0;
			}
		}
		goto parse;
	}else{
		ratta_mt_add_record(tch, jiffs);
		ratta_device->data_record[0][tch[SLIDER_TCH_T]][record_num].done = FINGER_DROP;
		//goto parse; need?
	}
	return 0;
parse:
	ratta_mt_parse(tch[SLIDER_TCH_T]);
	return 0;
}

int ratta_mt_probe(struct device *dev)
{
	int i, j, type, ret;
	printk("%s: >>>>>> %s <<<<<<\n", __func__,SLIDER_DRV_VER);

	//if (ratta_device) {
	//	return -EEXIST;
	//}

	ratta_device = (typeof(ratta_device))kzalloc(sizeof(*ratta_device),
						     GFP_KERNEL);
	if (!ratta_device) {
		return -ENOMEM;
	}

	ratta_device->input = devm_input_allocate_device(dev);
	if (!ratta_device->input) {
		ret = -ENOMEM;
		goto err;
	}
	for(type = 0; type < 2; type++) {
		for (i = 0; i < RATTA_MAX_FINGERS; i++) {
			for (j = 0; j < RATTA_MAX_REC; j++) {
				ratta_device->data_record[type][i][j].x = RATTA_INVALID_VALUE;
				ratta_device->data_record[type][i][j].y = RATTA_INVALID_VALUE;
				ratta_device->data_record[type][i][j].t = RATTA_INVALID_VALUE;
				ratta_device->data_record[type][i][j].e = RATTA_INVALID_VALUE;
				ratta_device->data_record[type][i][j].o = RATTA_INVALID_VALUE;
				ratta_device->data_record[type][i][j].tip = RATTA_INVALID_VALUE;
				ratta_device->data_record[type][i][j].num = RATTA_INVALID_VALUE;
				ratta_device->data_record[type][i][j].time = 0;
				ratta_device->data_record[type][i][j].done = 0;
			}
			ratta_device->data_record[type][i][0].status = 0;
			ratta_device->rec_now[type][i] = 0;
		}
	}
	ratta_device->slider_left_count = 0;
	ratta_device->slider_right_count = 0;
	ratta_device->input->name = RATTA_MT_NAME;
	ratta_device->input->id.bustype = BUS_I2C;

	// 20210717: 如果没有 VID/PID，则默认会通过 名字来匹配。
	//ratta_device->input->id.vendor = 0x0777;
	//ratta_device->input->id.product = 0x0007;
	// 20210717: 如果增加了 version，那么 kl 文件的后缀也要增加version。看
	// frameworks/native/libs/input/InputDevice.cpp 文件。
	//ratta_device->input->id.version = 0x0100;
	
	__set_bit(EV_KEY, ratta_device->input->evbit);
	__set_bit(KEY_LEFT, ratta_device->input->keybit);
	__set_bit(KEY_RIGHT, ratta_device->input->keybit);
	for(i=0;i<SLIDER_R_DOWN+1;i++){
		__set_bit(key_map[i], ratta_device->input->keybit);
	}
	ret = input_register_device(ratta_device->input);
	if (ret < 0) {
		goto err;
	}

	return 0;
err:
	kfree(ratta_device);
	ratta_device = NULL;

	return ret;
}

void ratta_mt_remove(void)
{
	input_unregister_device(ratta_device->input);
	kfree(ratta_device);
	ratta_device = NULL;
}
