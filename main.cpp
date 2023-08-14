//------------------------------------------------------------------//
//Supported MCU :  RZ/A1H
//File Contents :  kit20_gr-peach ( Trace Program )
//Version number:  Ver.1.00
//Date:            2020.09.01
//Copyright:       Renesas Electronics Corporation
//                 Hitachi Document Solutions Co., Ltd.
//------------------------------------------------------------------//

//This program supports the following kit:
//* M-S348 Image processing micon car production kit

/**********************************************************************/
/*
 * インクルード
 */
#include "mbed.h"
#include "iodefine.h"
#include "DisplayBace.h"
#include "ImageProcessFunc.h"
#include "EasyAttach_CameraAndLCD.h"

// add Include SD program
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <stdlib.h>

#include "FATFileSystem.h"
#include "SDBlockDevice_GRBoard.h"

/**********************************************************************/
/*
 * Define
 */

/**
 * 足回りモーター PWM周期
 * 
 * Motor PWM period
 * 1ms    P0φ/1  = 0.03us
 */
#define		MOTOR_PWM_CYCLE		33332

/**
 * ステアサーボモーター PWM周期
 * 
 * SERVO PWM period
 * 16ms   P0φ/16 = 0.48us
 */
#define		SERVO_PWM_CYCLE		33332

/**
 * サーボセンター値
 * 
 * 1.5ms / 0.48us - 1 = 3124
 * 値を足すと右　減らすと左
 */
#define		SERVO_CENTER		3124

/**
 * サーボ1°あたりのソフト内の値
 */
#define		HANDLE_STEP			18

/**
 * 二値化用の閾値
 * 
 * Set 0 to 255 Black:0 White:255
 */
#define		THRESHOLD			128

/**
 * 二値化したセンサのマスク定義
 * 
 * センサ確認範囲
 * ●：確認が必要ない場所
 * 〇：確認が必要な場所
 */
#define		MASK2_2		0x66		// ●〇〇● ●〇〇●
#define		MASK2_0		0x60		// ●〇〇● ●●●●
#define		MASK0_2		0x06		// ●●●● ●〇〇●
#define		MASK3_3		0xe7		// 〇〇〇● ●〇〇〇
#define		MASK0_3		0x07		// ●●●● ●〇〇〇
#define		MASK3_0		0xe0		// 〇〇〇● ●●●●
#define		MASK4_0		0xf0		// 〇〇〇〇 ●●●●
#define		MASK0_4		0x0f		// ●●●● 〇〇〇〇
#define		MASK4_4		0xff		// 〇〇〇〇 〇〇〇〇

/**
 * GR-PEACH基板のRGBLED制御用
 */
#define		LED_OFF		0x00
#define		LED_RED		0x01
#define		LED_GREEN	0x02
#define		LED_YELLOW	0x03
#define		LED_BLUE	0x04
#define		LED_PURPLE	0x05
#define		LED_SKYBLUE	0x06
#define		LED_WHITE	0x07

/**
 * LED制御用関数｢led_m_set｣用
 */
#define		RUN			0x00
#define		STOP		0x01
#define		ERROR		0x02
#define		DEBUG		0x03
#define		CRANK		0x04
#define		LCHANGE		0x05
#define		START_BAR	0x06
#define		LOG_WRITE	0x07

/**
 * 画像データシリアル出力関数｢SerialOut_CsvJpg｣用
 */
#define		COLOR			0x00
#define		BINARY			0x01
#define		COLOR_BINARY	0x02

/**
 * NTSC-Video関係のDefine
 */
#define		VIDEO_INPUT_CH	(DisplayBase::VIDEO_INPUT_CHANNEL_0)
#define		VIDEO_INT_TYPE	(DisplayBase::INT_TYPE_S0_VFIELD)

/**
 * ビデオフォーマット指定
 * 
 * VIDEO_YCBCR422 or VIDEO_RGB888
 */
#define		VIDEO_FORMAT	(VIDEO_YCBCR422)
//#define		VIDEO_FORMAT	(VIDEO_RGB888)

/**
 * 1ピクセル当たりのサイズ指定
 * 
 * ｢VIDEO_FORMAT｣の選択により決定する
 */
#if (VIDEO_FORMAT == VIDEO_YCBCR422)
#define DATA_SIZE_PER_PIC (2u)
#endif
#if (VIDEO_FORMAT == VIDEO_RGB888)
#define DATA_SIZE_PER_PIC (4u)
#endif

/**
 * 画像データ取り込み用のバッファサイズ指定
 * 
 * 32 か128の倍数に指定
 */
#define		PIXEL_HW				(160u)		// QQVGA
#define		PIXEL_VW				(120u)		// QQVGA
#define		VIDEO_BUFFER_STRIDE		(((PIXEL_HW * DATA_SIZE_PER_PIC) + 31u) & ~31u)
#define		VIDEO_BUFFER_HEIGHT		(PIXEL_VW)

/**
 * 
 */
#define		TOP			0	// Even
#define		BOTTOM		1	// Odd

/**
 * 画像データコピーサイズ
 * 
 * HALF or FULL
 */
#define		ICS		(HALF)
//#define		ICS		(FULL)

/**
 * トレースライン特定プログラム用
 */
#define		CAMERA_LINE_Y	60u		// 縦列の数
#define		CAMERA_LINE_X	80u		// 横列の数
#define		TRACE_LINE		20u		// 使う行16
#define		HL_NMBER		50u		// デバッグ画面で使う行
#define		HINDEX			1.46f	// ハンドルの指数(HANDLEINDEX) 1.46

/**
 * トレース用のゲイン関係
 */
#define		RATE			1		// 画面圧縮率
#define		THGAIN			0.3		// 閾値の作成のためのゲイン(centerLine用)
#define		HLGAIN			0.6		// 閾値の作成のためのゲイン(halfLine用)
#define		YRGAIN			0.18	// YRangeGain
#define		XRGAIN			0.87	// XRangeGain
#define		HAGAIN			0.69	// ハンドルのゲイン 0.10
#define		HANDEX_GAIN		0.3		// 指数関数時のハンドルのゲイン
#define		MOGAIN_GAIN		0.01	// この値が大きいほどカーブの時に内輪と外輪で差が大きくなる0.012
#define		HA_P_GAIN		0.5		// ハンドル制御用Pゲイン
#define		HA_D_GAIN		0.4		// ハンドル制御用Dゲイン

/**
 * 閾値関係
 */
#define		WHT_TH				 50		// 簡易ハーフ、クロスラインチェックでの閾値
#define		TIMEOUTVAL_CURVE	200		// 一度カーブを認識したら←ミリ秒は認識
#define		BLACK_TH			 10		// 閾値以下だったらLineなし
#define		HALF_TH				 80		// ハーフラインの閾値
#define		FRONT_HALF_TH		 70		// ハーフラインの閾値
#define		CURVE_HA_TH			 10		// カーブの閾値(ハンドルの値と比較)
#define		TOLERANCE_VAL		  3		// 画面の中心から白線がずれても動作が発生しない許容差

/**
 * クランクトレース速度(mm/s)
 */
#define		TRACE_MAX_PWM	80

/**
 * クランクトレース速度(mm/s)
 */
#define		CRANK_SPEED		25

/**
 * レーントレース速度(mm/s)
 */
#define		LANE_SPEED		30

/**
 * ログ保存画素サイズ
 */
#define		LOG_GASO_BYTES	(20 * 40)

/**
 * ログ保存数
 */
#define		LOG_NUM			(4000u)

/**
 * 走行終了時間(秒)
 */
#define		STOP_TIME		10L

/**
 * 走行終了距離(m)
 */
#define		STOP_DIST		1L


/**********************************************************************/
/*
 * 型定義
 */

/**
 * 二値化用センサ構造体
 */
typedef struct
{
	int				x[8];	// 二値化する座標 X
	int				y;		// 二値化する座標 Y
	unsigned char	v;		// 二値化後のビット値
} SENSOR;

/**
 * トレースライン検出用 1
 */
typedef struct
{
	int camera;		// カメラの画素値
	int dif_LTR;	// 左から右を引いた差分
	int dif_RTL;	// 右から右を引いた差分
} DIFF_DATA;

/**
 * トレースライン検出用 3
 * @note センターポイントに関する情報
 */
typedef struct
{
	int16_t topPoint_LTR;	// left to rightの略
	int16_t topPoint_RTL;	// right to leftの略
	int16_t Point;
} data3;

/**
 * トレースライン検出用 4
 */
typedef struct
{
	int topDif;
	int topCamera;		// カメラのデータの最大値
	int th;				// 閾値
	int cameraTh;
	int16_t topCnt_LTR;
	int16_t topCnt_RTL;
	int16_t centerCnt;
	int No;
	int PointDif;
	bool isLineTrue;
	int distance;
	uint WHTcnt;		// 白いマスがいくつあるか。
} data4;

/**
 * トレースライン検出用 5
 */
typedef struct
{
	uint checkcnt;
	uint trueCnt;
	uint ratio;
} HALF_LINE_CHECK;

/**
 * トレースライン検出用 6
 */
typedef struct
{
	int xRange;
	int yRange;
	int Th;			// ハーフラインとクロスラインの閾値
} data6;

/**
 * エンコーダー情報
 */
typedef struct
{
	uint16_t count10ms;		// 10msのカウンタ
	uint64_t totalCount;	// 合計のカウンタ
	uint64_t distCount;		// 減算用のカウンタ
	uint16_t mpsSpeed;		// ?(m/s)の値
	uint16_t speed;			// ?(m/s) ×10の値(速度制御用)
	uint16_t mTotalDist;	// 走行距離(m)
	uint16_t mmTotalDist;	// 走行距離(mm)
} ENCODER_INFO;

/**
 * ログ保存用構造体
 */
typedef struct
{
	unsigned char	pattern;		// 走行パターン
	unsigned int	sens;			// 二値化下センサ状態
	unsigned int	cnt_stop;		// 走行時間
	signed int		handle;			// ステアサーボ指定値
	signed int		speedL;			// 左モーター値
	signed int		speedR;			// 右モーター値
	signed char		center_diff;	// センターからのズレ値
	unsigned int	flag1;			// フラグ1値
	unsigned int	flag2;			// フラグ2値
	unsigned char	center_position[30];	// センター検出プログラム用
	unsigned char	gaso[LOG_GASO_BYTES];	// 画素データ
} SDLOG_T;


/**********************************************************************/
/*
 * プロトタイプ宣言
 */

/*
 * ログ関連
 */
void LOG_rec(mcrMat *ImData);
void SD_LOG_Write(void);

/*
 * ビデオ関係
 */
static void Start_Video_Camera(void);
static void IntCallbackFunc_Vfield(DisplayBase::int_type_t int_type);

/*
 * モーター初期化関数
 */
void init_MTU2_PWM_Motor(void); /* Initialize PWM Functions */
void init_MTU2_PWM_Servo(void); /* Initialize PWM Functions */

/*
 * エンコーダー初期化関数
 */
void initEncoder(void);

/*
 * 割り込み
 */
void intTimer(void);
void intImagePro(void);

/*
 * GR-PEACH基板制御用
 */
void led_m_rgb(int led);
void led_m_user(int led);
unsigned char user_button_get(void);
void led_m_set(int set);
void led_m_pro(void);

/*
 * モータードライブ基板制御用
 */
void led_out(int led);
unsigned char pushsw_get(void);
void motor(int accele_l, int accele_r);
void handle(int angle);
/*
 * シールド基板制御用
 */
unsigned char dipsw_get(void);

/*
 * センサ関係
 */
int initSensor(mcrMat *ImData, SENSOR *S, int PL);
void SensorPro(mcrMat *ImData, SENSOR *S); /* Only Function for interrupt */
unsigned char sensor_inp(SENSOR *S, unsigned char mask);
int startbar_get(void);
//int check_crossline(void);
//int check_rightline(void);
//int check_leftline(void);

/*
 * 画像処理関係
 */
inline char getImage(int x, int y);			// 画素値取得
inline char getShrinkImage(int x, int y);	// 圧縮画像の指定画素取得
inline void getShrinkImageSize(uint16_t &x, uint16_t &y);	// 圧縮画像のサイズ
void setDiffData(void);						// 圧縮画素データから差分値を計算
void getThreshold(void);					// 閾値計算
// 二値化変換
unsigned char convertBinarization(int line, int threshold, int diff);
unsigned long convertCharToLong(unsigned char hex);

/*
 * トレースライン検出
 */
void getCenterLine(void);		// センターライン取得
void findBorder(void);			// 境界線取得
void topAssociation(void);		// 境目と境目を関連付け
void getCenterCandidate(void);	// センターライン候補取得
void getCenterNumber(void);		// 各行のセンターライン検出
bool checkAssociation(int y, int maxDif, int minDif, int i_LTR, int i_RTL);

/*
 * トレース用
 */
int makeHandleVal(const int* linePoint);				// サーボの値作成
void autoRun(const int handleVal, const int power);		// サーボの値に呼応して変化

/*
 * マーカー検出
 */
void getLine_pe(void);			// ハーフラインの%算出
void getFrontLine_pe(void);		// ハーフラインの%算出
bool simpleLineCheck(int y);	// 簡単なラインチェック
void makeRange(void);			// ハーフランの範囲取得
int rangCcheck_y(int yData);
int checkReturnLine(void);		// 復帰条件
void checkLine(int yData, int &leftRate, int &rightRate);
void checkFrontLine(int yData, int &leftRate, int &rightRate);
inline int checkCrossLine(int lineLeftMax, int lineRightMax);
inline int checkFrontCrossLine(int lineLeftMax, int lineRightMax);

/*
 * デバッグ用
 */
void SerialOut_TeraTerm(mcrMat *ImData, int Mode);
void SerialOut_Excel(mcrMat *ImData, int Mode);
void SerialOut_CsvJpg(mcrMat *ImData, int Format, int ColorPattern);
void DebugPro(void);


/**********************************************************************/
/*
 * 変数宣言
 */

/*
 * ログ関係
 */
unsigned int log_no = 0;
volatile bool sdlog_enable = 0;
volatile bool sd_enable = 0;
volatile unsigned long cnt_stop;
int angle_buff;
int motor_buff_l, motor_buff_r;
SDLOG_T log_data[LOG_NUM];
FATFileSystem fs("storage");
SDBlockDevice_GRBoard sd;
static FILE *fp = NULL;

/*
 * DisplayBaseコンストラクタ
 */
DisplayBase Display;

/*
 * Tickerコンストラクタ
 */
Ticker interrput;

/*
 * Serialコンストラクタ
 */
Serial pc(USBTX, USBRX);

/*
 * GR-PEACH IO関係
 */
DigitalOut	LED_R(P6_13);		// LED1 on the GR-PEACH board
DigitalOut	LED_G(P6_14);		// LED2 on the GR-PEACH board
DigitalOut	LED_B(P6_15);		// LED3 on the GR-PEACH board
DigitalOut	USER_LED(P6_12);	// USER_LED on the GR-PEACH board
DigitalIn	user_botton(P6_0);	// SW1 on the GR-PEACH board

/*
 * シールド IO関係
 */
BusIn	dipsw(P7_15, P8_1, P2_9, P2_10);	// SW1 on Shield board

/*
 * モータードライブ IO関係
 */
DigitalOut	Left_motor_signal(P4_6);	// Used by motor Function  
DigitalOut	Right_motor_signal(P4_7);	// Used by motor Function  
DigitalIn	push_sw(P2_13);				// SW1 on the Motor Drive board
DigitalOut	LED_3(P2_14);				// LED3 on the Motor Drive board
DigitalOut	LED_2(P2_15);				// LED2 on the Motor Drive board

/*
 * NTSC-Video関係
 */
static uint8_t FrameBuffer_Video_A[VIDEO_BUFFER_STRIDE * VIDEO_BUFFER_HEIGHT] __attribute((section("NC_BSS"), aligned(16))); //16 bytes aligned!;
uint8_t *write_buff_addr = FrameBuffer_Video_A;
static volatile int32_t field = BOTTOM;
static volatile int32_t field_buff = BOTTOM;

/*
 * 画像関係
 */
mcrMat ImgInp0;		// Screen Size ( 160 * 120 )
mcrMat ImgInp1;		// Screen Size (  80 *  60 )
mcrMat ImgBuff;		// Screen Size (  80 *  60 )

/*
 * 二値化センサー関係
 */
SENSOR sensor0;		// Line trace sensor
SENSOR sensor1;		// Start bar sensor

/*
 * トレースライン検出用
 */
DIFF_DATA diffData[CAMERA_LINE_Y][CAMERA_LINE_X];
int16_t topDataLeftToRight[CAMERA_LINE_Y][CAMERA_LINE_X];
int16_t topDataRightToLeft[CAMERA_LINE_Y][CAMERA_LINE_X];
data3 lineCData[CAMERA_LINE_Y][CAMERA_LINE_X];
data4 lineData[CAMERA_LINE_Y];
data6 HL_Data[CAMERA_LINE_Y];			// 各行の長さ(halfLineRange)

/*
 * トレース関係のフラグ
*/
static bool isCurve = false;
static bool isHalfLine = false;
static bool isHalfLine_L = false;
static bool isHalfLine_R = false;
static bool isCrossLine = false;

static bool isfrontHalfLine_L = false;
static bool isfrontHalfLine_R = false;
static bool isfrontCrossLine = false;
static bool isfrontLine = false;	// centerlineの判別

static bool isUpdateLine = false;
static bool isSlopeUnder = false;

volatile unsigned long cnt_slope_under = 0;

static int cameraCenter = ((CAMERA_LINE_X - 2) / 2) + 1;		// カメラの中心(少なくすると左)
static int centerPoint[CAMERA_LINE_Y];							// センターラインの位置配列
static int senterPoint_D[CAMERA_LINE_Y][CAMERA_LINE_X];			// デバッグ用
static int handleVal;			// 切れ角
static int canDif;				// 許容できる差
// cameraのcenterを意図的にずらす時に使う(＋なら右)
static int traceLineOffsetVal = 0;
static uint16_t centerDiffAve = 0;

static float frontLinePe = 0;
static float lastCameraAverage = 0;
static float difCameraAverage = 0;
static float difCameraAverage_max = 0;
static float last_raw_handleVal = 0;

/*
 * トレース処理関係
 */
volatile unsigned long PNumber; // intTimer Function Only

volatile unsigned long cnt0;	// Used by timer Function
volatile unsigned long cnt1;	// Used within main
volatile int pattern = 0;		// Pattern numbers
volatile int initFlag = 1;		// Initialize flag

volatile int ThresholdBuff;		// ImageBinary Function only
volatile int ThresholdMax;		// ImageBinary Function only
volatile int ThresholdMin;		// ImageBinary Function only

volatile int LedPattern;		// led_m_pro Function only

static unsigned long cnt_Not_Rain_Change = 0;
static unsigned long cnt_curve;		// 誤検知防止
static uint64_t mainCount;
static int16_t maxPwm;

static uint16_t imgSizeX = CAMERA_LINE_X, imgSizeY = CAMERA_LINE_Y;	// 撮影画像のサイズ

/*
 * エンコーダー関係
 */
ENCODER_INFO encoderInfo = {0, 0, 0, 0, 0, 0, 0};

/*
 * デバッグ用
 */
static int debugMode;

/*
 * GR-PEACHのLED制御用
 */
const int led_data[][3] = {
/*	Color,		onTime,	offTime		*/
	{LED_GREEN,		500,	500},	// 0:RUN
	{LED_RED,		500,	  0},	// 1:STOP
	{LED_RED,		100,	100},	// 2:ERROR
	{LED_BLUE,		 50,	 50},	// 3:DEBUG
	{LED_YELLOW,	500,	500}, 	// 4:CRANK
	{LED_BLUE,		500,	500},	// 5:LCHANGE
	{LED_BLUE,		 50,	 50},	// 6:START_BAR
	{LED_RED,		100,	100},	// 7:LOG_WRITE
};


/**********************************************************************/
/*
 * メイン関数
 */
int main(void)
{
	// 初期化フラグ
	initFlag = 1;

	// 起動時にボタンが押されていた場合デバッグモードへ移行
	if (user_button_get() != 0)
	{
		debugMode = 1;
	}
	else
	{
		debugMode = 0;
	}

	// 足回りモーター初期化
	init_MTU2_PWM_Motor();
	// ステアサーボ初期化
	init_MTU2_PWM_Servo();
	// エンコーダー初期化
	initEncoder();
	// シリアル通信速度指定
	pc.baud(230400);

	// タイマー割り込み初期化(1ms)
	interrput.attach(&intTimer, 0.001);

	cnt1 = 0;
	makeRange();

	// 車体状態の初期化
	handle(0);
	motor(0, 0);
	led_out(0x0);
	led_m_set(STOP);

	// 閾値の初期化
	ThresholdBuff = THRESHOLD;

	// カメラの初期化
	EasyAttach_Init(Display, PIXEL_HW, PIXEL_VW);
	// 撮影開始
	Start_Video_Camera();

	// NTSC信号の待機(約200ms)
	wait(0.2);

	// シリアル画面クリア
	pc.printf("\033[2J");
	pc.printf("\033[50F");

	// スタートバー感知用センサの初期化
	sensor1.y = (int)ImgInp1.h - 5;
	sensor1.x[3] = (ImgInp1.w / 2) - ((ImgInp1.w / 8) / 2);
	sensor1.x[2] = sensor1.x[3] - (ImgInp1.w / 8);
	sensor1.x[1] = sensor1.x[2] - (ImgInp1.w / 8);
	sensor1.x[0] = sensor1.x[1] - (ImgInp1.w / 8);
	sensor1.x[4] = (ImgInp1.w / 2) + ((ImgInp1.w / 8) / 2);
	sensor1.x[5] = sensor1.x[4] + (ImgInp1.w / 8);
	sensor1.x[6] = sensor1.x[5] + (ImgInp1.w / 8);
	sensor1.x[7] = sensor1.x[6] + (ImgInp1.w / 8);

	// トレース用センサ初期化
	sensor0.y = 30;
	sensor0.x[3] = (ImgInp1.w / 2) - ((ImgInp1.w / 8) / 2);
	sensor0.x[2] = sensor0.x[3] - (ImgInp1.w / 8);
	sensor0.x[1] = sensor0.x[2] - (ImgInp1.w / 8);
	sensor0.x[0] = sensor0.x[1] - (ImgInp1.w / 8);
	sensor0.x[4] = (ImgInp1.w / 2) + ((ImgInp1.w / 8) / 2);
	sensor0.x[5] = sensor0.x[4] + (ImgInp1.w / 8);
	sensor0.x[6] = sensor0.x[5] + (ImgInp1.w / 8);
	sensor0.x[7] = sensor0.x[6] + (ImgInp1.w / 8);

	pc.printf("sd connect\n\r");
	// SDカードコネクト
	if (sd.connect())
	{
		fs.mount(&sd);
		sd_enable = true;
		pc.printf("SD SUCCESS &s\n\r", fs.getName());
	}
	else
	{
		sd_enable = false;
		pc.printf("SD ERROR");
	}

	// ログファイルオープン
	fp = fopen("/storage/test.csv", "w+");
	//fp = fopen("/test1.txt", "w");
	pc.printf("OPEN\r\n");
	if (fp != NULL)
	{
		sd_enable = true;
		pc.printf("SD FILE OPEN SUCCESS\n\r");
		fclose(fp);
	}
	else
	{
		sd_enable = false;
		led_m_set(DEBUG);
		pc.printf("SD FILE OPEN ERROR\n\r");
	}

	initFlag = 0;
	pc.printf("start\n\r");
	pc.printf("\033[2J");

	// デバッグ用プログラム
	if (debugMode == 1)
	{
		led_m_set(DEBUG);
		DebugPro();
	}

	while (1)
	{
		mainCount++;
		// 画像サイズの取得
		getShrinkImageSize(imgSizeX, imgSizeY);

		/**
		 * 走行パターン / センサ状態 等の表示
		 * 確認用(散歩モード)
		 * @note デバッグモード以外では使用しない!!
		 */
		if (debugMode == 1)
		{
			// 通常表示に戻す
			pc.printf("\033[49m");
			// 受信確認
			if (pc.readable())
			{
				if (pc.getc() == 'w')
				{
					maxPwm = 70;
				}
				else
				{
					maxPwm = 0;
				}
			}
			pc.printf("pattern %4d\r\n", pattern);
			pc.printf("max pwm %4d\r\n", maxPwm);
			pc.printf("sensor1 val:0x%02X / ", sensor1.v);
			pc.printf("sensor0 val:0x%02X\r\n", sensor0.v);
			pc.printf("DiffCenter %3d\r\n", cameraCenter - centerDiffAve);
			pc.printf("encoder speed %3d, ", encoderInfo.count10ms);
			pc.printf("%3d\r\n", encoderInfo.speed);
			pc.printf("total dist %5d, ", encoderInfo.totalCount);
			pc.printf("%6d(m), ", encoderInfo.mTotalDist);
			pc.printf("%6d(mm)\r\n", encoderInfo.mmTotalDist);
			// pc.printf("dipsw 0x%04X\r\n", dipsw_get());
			pc.printf("\033[H");
		}
		else
		{
			maxPwm = TRACE_MAX_PWM;
		}

		// 走行終了処理
		if (pattern >= 10 && 
		   (/*cnt_stop > STOP_TIME * 1000		||*/
			encoderInfo.mTotalDist >= STOP_DIST	||
			user_button_get()))
		{
			motor(0, 0);
			led_m_set(LOG_WRITE);
			led_out(0x1);
			cnt1 = 0;
			sdlog_enable = 0;
			SD_LOG_Write();
			led_out(0x2);
			cnt_stop = 0;
			led_m_set(ERROR);
			pattern = 0;
		}

		// ログ開始条件
		if (pattern >= 10)
		{
			sdlog_enable = 1;
		}

		/**
		 * トレースプログラム
		 * 走行パターンの説明
		 * 
		 *  0：RUNスイッチ待機
		 *  1：スタートバーオープン待ち
		 * 11：通常トレース
		 * 12：通常トレース(モーターパワー：0)
		 * 21：クロスライン感知
		 * 22：クロスライン誤読み防止
		 * 23：クロスライン後のトレース処理
		 * 31：左クランク処理
		 * 32：左クランクから復帰処理
		 * 41：右クランク処理
		 * 42：右クランクから復帰処理
		 * 51：右ハーフライン感知
		 * 52：右ハーフライン誤読み防止
		 * 53：右ハーフライン後のトレース処理
		 * 54：右レーンチェンジ処理
		 * 61：左ハーフライン感知
		 * 62：左ハーフライン誤読み防止
		 * 63：左ハーフライン後のトレース処理
		 * 64：左レーンチェンジ処理
		 */
		switch (pattern)
		{
		/**
		 * RUNスイッチ待機
		 */
		case 0:
			// RUNスイッチ確認
			if (pushsw_get())
			{
				led_out(0x0);
				pattern = 1;
				while (pushsw_get());
				cnt1 = 0;
				break;
			}
			// スタートバー確認
			if (!startbar_get())
			{
				// LED点滅
				led_m_set(STOP);
			}
			else
			{
				// LED点滅
				led_m_set(START_BAR);
			}
			cnt_stop = 0;
			// ハンドル止める動作
			handle(0);
			motor(  0,  0);
			// トレースする位置設定
			cameraCenter = ((imgSizeX - 2) / 2);
			// 常にセンターとする
			for (uint8_t y = 0; y <= CAMERA_LINE_Y - 1; y++)
			{
				// 常に真ん中を入れる
				centerPoint[y] = (CAMERA_LINE_X - 2) / 2;
			}
			break;

		/**
		 * スタートバーオープン待ち
		 */
		case 1:
			// スタートバー確認
			if (!startbar_get())
			{
				led_out(0x0);
				pattern = 10;
				cnt1 = 0;
				cnt_stop = 0;
				// トータルカウンタクリア
				encoderInfo.totalCount = 0;
				break;
			}
			// LED点滅
			if (cnt1 < 50)
			{
				led_out(0x1);
			}
			else if (cnt1 < 100)
			{
				led_out(0x2);
			}
			else
			{
				cnt1 = 0;
			}
			break;

		/**
		 * 100ms間はとにかくまっすぐ走る
		 */
		case 10:
			handle(0);
			motor(maxPwm, maxPwm);
			if ( cnt1 >= 100 )
			{
				pattern = 11;
			}
			break;

		/**
		 * 通常トレース
		 */
		case 11:
			// クロスラインチェック
			if (isCrossLine && !isSlopeUnder)
			{
				pattern = 21;
				break;
			}
			// 右ハーフラインチェック
			if (isHalfLine_R && !isSlopeUnder)
			{
				pattern = 51;
				break;
			}
			// 左ハーフラインチェック
			if (isHalfLine_L && !isSlopeUnder)
			{
				pattern = 61;
				break;
			}
			// ライントレース
			autoRun(handleVal, maxPwm - difCameraAverage);
		//	autoRun(handleVal, 0);
// 旧プログラム
#if 0
			// クロスライン確認
			if (check_crossline())
			{
				pattern = 21;
				break;
			}
			// 右ハーフライン確認
			if (check_rightline())
			{
				pattern = 51;
				break;
			}
			// 左ハーフライン確認
			if (check_leftline())
			{
				pattern = 61;
				break;
			}

			// トレース処理(センサ確認範囲 -> 〇〇〇● ●〇〇〇)
			switch (sensor_inp(&sensor0, MASK3_3))
			{
			// 〇〇〇X X〇〇〇
			case 0x00:
				handle(0);
				motor(100, 100);
				break;
			// 〇〇〇X X●〇〇
			case 0x04:
				handle(5);
				motor(100, 100);
				break;
			// 〇〇〇X X●●〇
			case 0x06:
				handle(10);
				motor(80, 67);
				break;
			// 〇〇〇X X●●●
			case 0x07:
				handle(15);
				motor(50, 38);
				break;
			// 〇〇〇X X〇●●
			case 0x03:
				handle(25);
				motor(30, 19);
				pattern = 12;
				break;
			// 〇〇●X X〇〇〇
			case 0x20:
				handle(-5);
				motor(100, 100);
				break;
			// 〇●●X X〇〇〇
			case 0x60:
				handle(-10);
				motor(67, 80);
				break;
			// ●●●X X〇〇〇
			case 0xe0:
				handle(-15);
				motor(38, 50);
				break;
			// ●●〇X X〇〇〇
			case 0xc0:
				handle(-25);
				motor(19, 30);
				pattern = 13;
				break;

			default:
				break;
			}
#endif
			break;

		/**
		 * 通常トレース(モーターパワー：0)
		 */
		case 12:
			autoRun(handleVal, 0);
			pattern = 12;
			break;

		/**
		 * クロスライン感知
		 */
		case 21:
			led_out(0x3);
// 旧プログラム
#if 0
			handle(0);
			motor(0, 0);
#endif
			autoRun(handleVal, -40);
			pattern = 22;
			cnt1 = 0;
			// 誤読み対策距離 20mm
			encoderInfo.distCount = 22;
			break;

		/**
		 * クロスライン誤読み防止
		 */
		case 22:
			// エンコーダー仕様に変更
			if (cnt1 > 100 || !encoderInfo.distCount)
			{
				pattern = 23;
				cnt1 = 0;
				// 1mの距離カウント
				encoderInfo.distCount = 1092;
			}
			break;

		/**
		 * クロスライン後のトレース処理
		 */
		case 23:
			// 左ハーフライン検出
			if (isHalfLine_L)
			{
				// サーボ曲げ動作
				handle(43);
				motor( -20, 70);
				pattern = 31;
				cnt1 = 0;
				cnt_Not_Rain_Change = 0;
				break;
			}

			// 右ハーフライン検出
			if (isHalfLine_R)
			{
				// サーボ曲げ動作
				handle(-43);
				motor( 70, -20);
				pattern = 41;
				cnt1 = 0;
				cnt_Not_Rain_Change = 0;
				break;
			}
			// クランクキャンセル処理
			if (cnt1 >= 1000 || !encoderInfo.distCount)
			{
				pattern = 11;
			}

			// クランクトレース処理
			if (encoderInfo.speed < CRANK_SPEED)
			{
				autoRun(handleVal, 40);
			}
			else
			{
				if (encoderInfo.speed > CRANK_SPEED + 3)
				{
					autoRun(handleVal, -40);
				}
				else
				{
					autoRun(handleVal,   1);
				}
			}
// 旧プログラム
#if 0
			// 左マーカー確認
			// ●●●● ●〇〇〇
			if (sensor_inp(&sensor0, MASK4_4) == 0xf8)
			{
				led_out(0x1);
				handle(-38);
				motor(10, 50);
				pattern = 31;
				cnt1 = 0;
				break;
			}
			// 右マーカー確認
			// 〇〇〇● ●●●●
			if (sensor_inp(&sensor0, MASK4_4) == 0x1f)
			{
				led_out(0x2);
				handle(38);
				motor(50, 10);
				pattern = 41;
				cnt1 = 0;
				break;
			}

			// トレース処理(センサ確認範囲 -> 〇〇〇● ●〇〇〇)
			switch (sensor_inp(&sensor0, MASK3_3))
			{
			// 〇〇〇X X〇〇〇
			case 0x00:
				handle(0);
				motor(40, 40);
				break;
			// 〇〇〇X X●〇〇
			case 0x04:
			// 〇〇〇X X●●〇
			case 0x06:
			// 〇〇〇X X●●●
			case 0x07:
			// 〇〇〇X X〇●●
			case 0x03:
				handle(8);
				motor(40, 35);
				break;
			// 〇〇●X X〇〇〇
			case 0x20:
			// 〇●●X X〇〇〇
			case 0x60:
			// ●●●X X〇〇〇
			case 0xe0:
			// ●●〇X X〇〇〇
			case 0xc0:
				handle(-8);
				motor(35, 40);
				break;
			}
#endif
			break;

		/**
		 * 左クランク処理
		 */
		case 31:
			if (cnt1 > 200)
			{
				pattern = 32;
				cnt1 = 0;
			}
			break;

		/**
		 * 左クランク復帰処理
		 */
		case 32:
			// 復帰確認
			switch (sensor_inp(&sensor0, MASK4_4))
			{
			// ○●○○ ○○○○
			case 0x40:
			// ○●●○ ○○○○
			case 0x60:
			// ○○●○ ○○○○
			case 0x20:
			// ○○●● ○○○○
			case 0x30:
			// ○○○● ○○○○
			case 0x10:
			// ○○○● ●○○○
			case 0x18:
			// ○○○○ ●○○○
			case 0x08:
				led_out(0x0);
				pattern = 11;
				cnt1 = 0;
				break;
			}
#if 0
			// 〇●●X X〇〇〇
			if (sensor_inp(&sensor0, MASK3_3) == 0x60)
			{
				led_out(0x0);
				pattern = 11;
				cnt1 = 0;
			}
#endif
			break;

		/**
		 * 右クランク処理
		 */
		case 41:
			if (cnt1 > 200)
			{
				pattern = 42;
				cnt1 = 0;
			}
			break;

		/**
		 * 右クランク復帰処理
		 */
		case 42:
			// 復帰確認
			switch (sensor_inp(&sensor0, MASK4_4))
			{
			// ○○○○ ○○●○
			case 0x02:
			// ○○○○ ○●●○
			case 0x06:
			// ○○○○ ○●○○
			case 0x04:
			// ○○○○ ●●○○
			case 0x0c:
			// ○○○○ ●○○○
			case 0x08:
			// ○○○● ●○○○
			case 0x18:
			// ○○○● ○○○○
			case 0x10:
				led_out(0x0);
				pattern = 11;
				cnt1 = 0;
				break;
			}
// 旧プログラム
#if 0
			// 〇〇〇X X●●〇
			if (sensor_inp(&sensor0, MASK3_3) == 0x06)
			{
				led_out(0x0);
				pattern = 11;
				cnt1 = 0;
			}
#endif
			break;

		/**
		 * 右ハーフライン感知
		 */
		case 51:
			led_out(0x2);
			// 右側を走るようになる
			traceLineOffsetVal = 0;
			autoRun(handleVal, -20);
			isHalfLine = true;
			pattern = 52;
			cnt1 = 0;
			// 誤読み対策距離 20mm
			encoderInfo.distCount = 22;
			break;

		/**
		 * 右ハーフライン誤読み防止
		 */
		case 52:
			if (cnt1 > 100 || !encoderInfo.distCount)
			{
				pattern = 53;
				cnt1 = 0;
				// 1mの距離カウント
				encoderInfo.distCount = 1092;
			}
			// クロスライン確認
			if (isCrossLine)
			{
				isHalfLine = false;
				pattern = 21;
				break;
			}
			break;

		/**
		 * 右レーンチェンジ処理
		 */
		case 53:
			if (frontLinePe <= 10)
			{
				// トレース処理
				if (encoderInfo.speed < LANE_SPEED)
				{
					autoRun(handleVal, 40);
				}
				else
				{
					if (encoderInfo.speed > LANE_SPEED + 3)
					{
						autoRun(handleVal, -40);
					}
					else
					{
						autoRun(handleVal,   1);
					}
				}
			}
			// トレースライン無し
			else
			{
				handle(0);
				motor( 40, 40);
				pattern = 531;
				cnt1 = 0;
				break;
			}
			// レーンキャンセル処理
			if (cnt1 >= 1000 || !encoderInfo.distCount)
			{
				pattern = 11;
			}
#if 0
			// 全無反応待ち
			if (sensor_inp(&sensor0, MASK4_4) == 0x00)
			{
				handle(15);
				motor(40, 31);
				pattern = 54;
				cnt1 = 0;
				break;
			}

			// トレース処理(センサ確認範囲 -> 〇〇〇● ●〇〇〇)
			switch (sensor_inp(&sensor0, MASK3_3))
			{
			// 〇〇〇X X〇〇〇
			case 0x00:
				handle(0);
				motor(40, 40);
				break;
			// 〇〇〇X X●〇〇
			case 0x04:
			// 〇〇〇X X●●〇
			case 0x06:
			// 〇〇〇X X●●●
			case 0x07:
			// 〇〇〇X X〇●●
			case 0x03:
				handle(8);
				motor(40, 35);
				break;
			// 〇〇●X X〇〇〇
			case 0x20:
			// 〇●●X X〇〇〇
			case 0x60:
			// ●●●X X〇〇〇
			case 0xe0:
			// ●●〇X X〇〇〇
			case 0xc0:
				handle(-8);
				motor(35, 40);
				break;
			default:
				break;
			}
#endif
			break;

		/**
		 * 右レーンチェンジ処理
		 */
		case 531:
			// @TODO 待機(エンコーダー仕様に変更)
			if (cnt1 >= 130)
			{
				isCurve = true;
				handle(-10);
				motor( 45, 30);
				pattern = 54;
				cnt1 = 0;
				cnt_Not_Rain_Change = 0;
				break;
			}
			break;

		/**
		 * 右レーンチェンジ復帰動作
		 */
		case 54:
			// 復帰確認
			// @TODO 曲げ続ける(エンコーダー仕様に変更)
			if (cnt1 >= 500)
			{
				pattern = 55;
				cnt1 = 0;
				canDif = 18;
				// 常にセンターとする
				for (uint8_t y = 0; y <= CAMERA_LINE_Y - 1; y++)
				{
					centerPoint[y] = (CAMERA_LINE_X - 2) / 2;
				}
			}
#if 0
			// 〇〇●● ●●〇〇
			if (sensor_inp(&sensor0, MASK4_4) == 0x3c)
			{
				led_out(0x0);
				pattern = 11;
				cnt1 = 0;
			}
#endif
			break;

		/**
		 * 右レーンチェンジ終了確認
		 */
		case 55:
			// トレース処理
			autoRun(handleVal, 40);
			// 復帰ライン確認
			if (checkReturnLine())
			{
				// 常にセンターとする
				for (uint8_t y = 0; y <= CAMERA_LINE_Y - 1; y++)
				{
					centerPoint[y] = (CAMERA_LINE_X - 2) / 2;
				}
				pattern = 11;
				cnt1 = 0;
				canDif = 12;
			}
			break;

		/**
		 * 左ハーフライン感知
		 */
		case 61:
			led_out(0x1);
// 旧プログラム
#if 0
			handle(0);
			motor(0, 0);
#endif
			// 左側を走るようになる
			traceLineOffsetVal = 0;
			autoRun(handleVal, -20);
			isHalfLine = true;
			pattern = 62;
			cnt1 = 0;
			// 誤読み対策距離 20mm
			encoderInfo.distCount = 22;
			break;

		/**
		 * 左ハーフライン誤読み防止
		 */
		case 62:
			if (cnt1 > 100 || !encoderInfo.distCount)
			{
				pattern = 63;
				cnt1 = 0;
				// 1mの距離カウント
				encoderInfo.distCount = 1092;
			}
			// クロスライン確認
			if (isCrossLine)
			{
				isHalfLine = false;
				pattern = 21;
				break;
			}
			break;

		/**
		 * 左レーンチェンジ処理
		 */
		case 63:
			if (frontLinePe <= 10)
			{
				// トレース処理
				if (encoderInfo.speed < LANE_SPEED)
				{
					autoRun(handleVal, 40);
				}
				else
				{
					if (encoderInfo.speed > LANE_SPEED + 3)
					{
						autoRun(handleVal, -40);
					}
					else
					{
						autoRun(handleVal,   1);
					}
				}
			}
			// トレースライン無し
			else
			{
				handle(0);
				motor( 40, 40);
				pattern = 631;
				cnt1 = 0;
			}
			// レーンキャンセル処理
			if (cnt1 >= 1000 || !encoderInfo.distCount)
			{
				pattern = 11;
			}
#if 0
			// 全無反応待ち
			if (sensor_inp(&sensor0, MASK4_4) == 0x00)
			{
				handle(-15);
				motor(31, 40);
				pattern = 64;
				cnt1 = 0;
				break;
			}

			// トレース処理(センサ確認範囲 -> 〇〇〇● ●〇〇〇)
			switch (sensor_inp(&sensor0, MASK3_3))
			{
			// 〇〇〇X X〇〇〇
			case 0x00:
				handle(0);
				motor(40, 40);
				break;
			// 〇〇〇X X●〇〇
			case 0x04:
			// 〇〇〇X X●●〇
			case 0x06:
			// 〇〇〇X X●●●
			case 0x07:
			// 〇〇〇X X〇●●
			case 0x03:
				handle(8);
				motor(40, 35);
				break;
			// 〇〇●X X〇〇〇
			case 0x20:
			// 〇●●X X〇〇〇
			case 0x60:
			// ●●●X X〇〇〇
			case 0xe0:
			// ●●〇X X〇〇〇
			case 0xc0:
				handle(-8);
				motor(35, 40);
				break;
			default:
				break;
			}
#endif
			break;

		/**
		 * 左レーンチェンジ処理
		 */
		case 631:
			// @TODO 待機(エンコーダー仕様に変更)
			if (cnt1 >= 130)
			{
				isCurve = true;
				handle(10);
				motor( 30, 45);
				pattern = 64;
				cnt1 = 0;
				cnt_Not_Rain_Change = 0;
				break;
			}
			break;

		/**
		 * 左レーンチェンジ復帰動作
		 */
		case 64:
			// 復帰確認
			// @TODO 曲げ続ける(エンコーダー仕様に変更)
			if (cnt1 >= 200)
			{
				pattern = 65;
				cnt1 = 0;
				canDif = 18;
				// 常にセンターとする
				for (uint8_t y = 0; y <= CAMERA_LINE_Y - 1; y++)
				{
					centerPoint[y] = (CAMERA_LINE_X - 2) / 2;
				}
			}
#if 0
			// 〇〇●● ●●〇〇
			if (sensor_inp(&sensor0, MASK4_4) == 0x3c)
			{
				led_out(0x0);
				pattern = 11;
				cnt1 = 0;
			}
#endif
			break;

		/**
		 * 左レーンチェンジ終了確認
		 */
		case 65:
			// トレース処理
			autoRun(handleVal, 40);
			// 復帰ライン確認
			if (checkReturnLine())
			{
				// 常にセンターとする
				for (uint8_t y = 0; y <= CAMERA_LINE_Y - 1; y++)
				{
					centerPoint[y] = (CAMERA_LINE_X - 2) / 2;
				}
				pattern = 11;
				cnt1 = 0;
				canDif = 12;
			}
			break;

		default:
			break;
		}
	}
}


/************************************************************************/
/**
 * 足回りモーター初期化.
 * 
 * @note MTU2_3, MTU2_4
 * Reset-Synchronized PWM mode
 * TIOC4A(P4_4)	:Left-motor
 * TIOC4B(P4_5)	:Right-motor
 */
void init_MTU2_PWM_Motor(void)
{
	// IOポートレジスタ設定
	// alternative mode

	// MTU2_4 (P4_4)(P4_5)
	GPIOPBDC4	 = 0x0000;		// ポート設定 双方向モードの禁止
	GPIOPFCAE4	&= 0xffcf;		// ピン出力設定
	GPIOPFCE4	|= 0x0030;		// ピン出力設定
	GPIOPFC4	&= 0xffcf;		// ポート兼用モード設定

	GPIOP4		&= 0xffcf;		//
	GPIOPM4		&= 0xffcf;		// p4_4,P4_5:output
	GPIOPMC4	|= 0x0030;		// P4_4,P4_5:double

	// Module stop 33(MTU2) canceling
	CPGSTBCR3	&= 0xf7;

	// MTU2_3 and MTU2_4 (Motor PWM)
	MTU2TCR_3	= 0x20;				// TCNT Clear(TGRA), P0φ/1
	MTU2TOCR1	= 0x04;				//
	MTU2TOCR2	= 0x40;				// N L>H  P H>
	MTU2TMDR_3	= 0x38;				// Buff:ON Reset-Synchronized PWM mode
	MTU2TMDR_4	= 0x30;				// Buff:ON
	MTU2TOER	= 0xc6;				// TIOC3B,4A,4B enabled output
	MTU2TCNT_3	= MTU2TCNT_4 = 0;	// TCNT3/4 クリア
	MTU2TGRA_3	= MTU2TGRC_3 = MOTOR_PWM_CYCLE;
	// PWM-周期(1ms)
	MTU2TGRA_4	= MTU2TGRC_4 = 0;	// Left-motor(P4_4)
	MTU2TGRB_4	= MTU2TGRD_4 = 0;	// Right-motor(P4_5)

	// カウンタ開始
	MTU2TSTR	|= 0x40;
}

/************************************************************************/
/**
 * サーボモーター初期化.
 * 
 * @note MTU2_0 PWMモード1
 * TIOC0A(P4_0) :Servo-motor
 */
void init_MTU2_PWM_Servo(void)
{
	// ポート設定

	// MTU2_0 (P4_0)
	GPIOPBDC4	 = 0x0000;		// 双方向モード無効
	GPIOPFCAE4	&= 0xfffe;		// ピン出力設定
	GPIOPFCE4	&= 0xfffe;		// ピン出力設定
	GPIOPFC4	|= 0x0001;		// ピン出力設定

	GPIOP4		&= 0xfffe;		//
	GPIOPM4		&= 0xfffe;		// p4_0:output
	GPIOPMC4	|= 0x0001;		// P4_0:double

	// Module stop 33(MTU2) canceling
	CPGSTBCR3 &= 0xf7;

	// MTU2_0 (Motor PWM)
	MTU2TCR_0	= 0x22;		// TCNT Clear(TGRA), P0φ/16
	MTU2TIORH_0	= 0x52;		// TGRA L>H, TGRB H>L
	MTU2TMDR_0	= 0x32;		// TGRC and TGRD = Buff-mode

	// PWM-mode1
	MTU2TCNT_0	= 0;		// TCNT0 Set 0
	MTU2TGRA_0	= MTU2TGRC_0 = SERVO_PWM_CYCLE;
	// PWM-Cycle(16ms)
	MTU2TGRB_0	= MTU2TGRD_0 = 0;	// Servo-motor(P4_0)

	// TCNT_0 スタート
	MTU2TSTR	|= 0x01;
}

/************************************************************************/
/**
 * エンコーダー初期化.
 */
void initEncoder(void)
{
	CPGSTBCR3 &= ~0x08;

	GPIOPIBC1  &= ~0x0401;
	GPIOPBDC1  &= ~0x0401;
	GPIOPM1    |= 0x0401;
	GPIOPMC1   &= ~0x0401;
	GPIOPIPC1  &= ~0x0401;

	GPIOPBDC1  &= ~0x0401;

	GPIOPFC1   |= 0x0400;
	GPIOPFCE1  |= 0x0401;

	GPIOPIPC1  |= 0x0401;
	GPIOPMC1   |= 0x0401;

	// カウンタ停止
	MTU2TSTR   &= ~0x02;
	// 外部カウンタ / 両エッジカウント に設定
	MTU2TCR_1   = 0x14;
	// 動作モード 通常に設定
	MTU2TMDR_1  = 0x00;
	MTU2TIOR_1 |= 0xAA;
	// カウンタ開始
	MTU2TSTR   |= 0x02;
}

/************************************************************************/
/**
 * カメラ初期化.
 */
static void Start_Video_Camera(void)
{
	// 背景を黒に初期化
	for (uint32_t i = 0; i < sizeof(FrameBuffer_Video_A); i += 2)
	{
		FrameBuffer_Video_A[i + 0] = 0x10;
		FrameBuffer_Video_A[i + 1] = 0x80;
	}

	// カメラの割り込みコールバック登録
	Display.Graphics_Irq_Handler_Set(VIDEO_INT_TYPE, 0, IntCallbackFunc_Vfield);
	// ビデオ撮影設定
	Display.Video_Write_Setting(
		VIDEO_INPUT_CH,
		DisplayBase::COL_SYS_NTSC_358,
		write_buff_addr,
		VIDEO_BUFFER_STRIDE,
#if (VIDEO_FORMAT == VIDEO_YCBCR422)
		DisplayBase::VIDEO_FORMAT_YCBCR422,
		DisplayBase::WR_RD_WRSWA_32_16BIT,
#endif
#if (VIDEO_FORMAT == VIDEO_RGB888)
		DisplayBase::VIDEO_FORMAT_RGB888,
		DisplayBase::WR_RD_WRSWA_32BIT,
#endif
		PIXEL_VW,
		PIXEL_HW);
	// 撮影スタート
	EasyAttach_CameraStart(Display, VIDEO_INPUT_CH);
}

/************************************************************************/
/**
 * カメラ撮影割り込みコールバック関数.
 *
 *	@param[in]		int_type		割り込みタイプ
 */
static void IntCallbackFunc_Vfield(DisplayBase::int_type_t int_type)
{
	(void)int_type;

	// top or bottom (Change)
	if (field == BOTTOM)
	{
		field = TOP;
	}
	else
	{
		field = BOTTOM;
	}
	// Led
	led_m_user(field);
}

/************************************************************************/
/**
 * タイマー割り込み関数 1ms.
 */
void intTimer(void)
{
	static uint16_t timer10ms = 0;
	mainCount = 0;
	cnt0++;
	cnt1++;
	cnt_stop++;
	cnt_slope_under++;
	cnt_Not_Rain_Change++;

	// コーナーカウンタ
	if (isCurve)
	{
		cnt_curve++;
	}

	// filed確認
	if (field_buff != field)
	{
		field_buff = field;
		PNumber = 0;
	}
	// 撮影画像処理
	intImagePro();
	// GR-PEACH LED制御
	led_m_pro();

	/**
	 * 10msタイマー処理
	 */
	timer10ms++;
	if (!(timer10ms % 10))
	{
		/**
		 * @TODO エンコーダー
		 * 
		 * 使用タイヤ：φ21(mm)
		 * 1周：72(パルス), 65.94(mm)
		 * ↓↓1m分の計算↓↓ 1091.9パルスで1m
		 * 72(パルス) : 65.94(mm) = 1091.9(パルス) : 1000(mm)
		 */
		// エンコーダー取得
		encoderInfo.count10ms = MTU2TCNT_1;
		// カウンタクリア
		MTU2TCNT_1 = 0;
		// トータルカウンタに加算
		encoderInfo.totalCount += encoderInfo.count10ms;
		// 距離フラグ減算
		if (encoderInfo.distCount)
		{
			encoderInfo.distCount -= encoderInfo.count10ms;
		}
		else
		{
			encoderInfo.distCount = 0;
		}

		/**
		 * 速度計算(速度制御用) = 1秒間のパルス数                 / 0.1mのパルス数
		 *						(10msのカウント数(パルス) * 100) / 109.19
		 */
		encoderInfo.speed    = (encoderInfo.count10ms * 100) / 109.19;
		encoderInfo.mpsSpeed = encoderInfo.speed / 10;

		/**
		 * 距離計算(mm) = 合計カウント / 1mmのパルス数
		 */ 
		encoderInfo.mmTotalDist = (encoderInfo.totalCount / 1.0919);

		/**
		 * 距離計算(m) = 距離計算(mm) / 1000
		 */ 
		encoderInfo.mTotalDist = encoderInfo.mmTotalDist / 1000;

		// ログ保存
		LOG_rec(&ImgInp1);
		// クリア
		timer10ms = 0;
	}
}

/************************************************************************/
/**
 * 画像処理.
 * @note 1ms割り込みから呼び出し
 */
void intImagePro(void)
{
	switch (PNumber++)
	{
#if (VIDEO_FORMAT == VIDEO_YCBCR422)
	// 撮影データを｢ImgInp0｣へコピー
	case 0:
		intImageCopy(write_buff_addr, PIXEL_HW, PIXEL_VW, VIDEO_FORMAT, field, &ImgInp0, ICS, INTERMITTENT_PRO4);
		break;
	case 1:
		intImageCopy(write_buff_addr, PIXEL_HW, PIXEL_VW, VIDEO_FORMAT, field, &ImgInp0, ICS, INTERMITTENT_PRO4);
		break;
	case 2:
		intImageCopy(write_buff_addr, PIXEL_HW, PIXEL_VW, VIDEO_FORMAT, field, &ImgInp0, ICS, INTERMITTENT_PRO4);
		break;
	case 3:
		intImageCopy(write_buff_addr, PIXEL_HW, PIXEL_VW, VIDEO_FORMAT, field, &ImgInp0, ICS, INTERMITTENT_PRO4);
		
		// 二値化用閾値自動計算
		ThresholdBuff = DiscriminantAnalysisMethod(&ImgInp0);
		// 最大閾値
		ThresholdMax = ThresholdBuff + 255;
		if (ThresholdMax > 255)
		{
			ThresholdMax = 255;
		}
		// 最小閾値
		ThresholdMin = ThresholdBuff + 0;
		if (ThresholdMin <= 50)
		{
			ThresholdMin = 50;
		}
		break;
	// 撮影画像を1/2に圧縮(Top field or Bottom field)
	// (160 * 120) -> (80 * 60)
	// 圧縮後のデータは｢ImgInp1｣へコピーされる
	case 4:
		intImageReduction(&ImgInp0, &ImgInp1, VIDEO_FORMAT, RATE, INTERMITTENT_PRO2);
		break;
	case 5:
		intImageReduction(&ImgInp0, &ImgInp1, VIDEO_FORMAT, RATE, INTERMITTENT_PRO2);
		break;
	case 6:
		// 画像データ処理
		ImageBinarySetY(ThresholdMin, ThresholdMax);
		// 画像データ二値化
		intImageBinary(&ImgInp1, &ImgInp1, BINARY_Y, INTERMITTENT_PRO1);
		// スタートバー位置二値化
		SensorPro(&ImgInp1, &sensor1);
		// トレース位置二値化
		SensorPro(&ImgInp1, &sensor0);
		break;
	case 7:
		// カメラデータの微分処理
		setDiffData();
		// 中央の白線感知
		getCenterLine();
		// ハンドルの値生成
		handleVal = makeHandleVal(centerPoint);
		// ハーフラン検出
		getLine_pe();
		getFrontLine_pe();
		break;

#endif

#if (VIDEO_FORMAT == VIDEO_RGB888)
	case 0:
		// Size( 160 * 120 ) -> ( 80 * 60 ) Top field or Bottom field
		intImageCopy(write_buff_addr, PIXEL_HW, PIXEL_VW, VIDEO_FORMAT, field, &ImgInp0, ICS, INTERMITTENT_PRO4);
		break;
	case 1:
		intImageCopy(write_buff_addr, PIXEL_HW, PIXEL_VW, VIDEO_FORMAT, field, &ImgInp0, ICS, INTERMITTENT_PRO4);
		break;
	case 2:
		intImageCopy(write_buff_addr, PIXEL_HW, PIXEL_VW, VIDEO_FORMAT, field, &ImgInp0, ICS, INTERMITTENT_PRO4);
		break;
	case 3:
		intImageCopy(write_buff_addr, PIXEL_HW, PIXEL_VW, VIDEO_FORMAT, field, &ImgInp0, ICS, INTERMITTENT_PRO4);
		break;
	case 4:
		// Size( 80 * 60 ) -> ( 40 * 30 )
		intImageReduction(&ImgInp0, &ImgInp1, VIDEO_FORMAT, Rate, INTERMITTENT_PRO2);
		break;
	case 5:
		intImageReduction(&ImgInp0, &ImgInp1, VIDEO_FORMAT, Rate, INTERMITTENT_PRO2);
		break;
	case 6:
		// RGB -> YCBCR422
		intRGB_YCBCR_Converter(&ImgInp1, INTERMITTENT_PRO1);
		break;
	case 7:
		// Automatic calculation of threshold value ( YCBCR422 Only function )
		ThresholdBuff = DiscriminantAnalysisMethod(&ImgInp1);
		// Threshold Max
		ThresholdMax = ThresholdBuff + 255;
		if (ThresholdMax > 255)
			ThresholdMax = 255;
		// Threshold Min
		ThresholdMin = ThresholdBuff + 0;
		if (ThresholdMin <= 50)
			ThresholdMin = 50;
		break;
	case 8:
		// Binary process
		ImageBinarySetR(ThresholdMin, ThresholdMax);
		ImageBinarySetG(ThresholdMin, ThresholdMax);
		ImageBinarySetB(ThresholdMin, ThresholdMax);
		intImageBinary(&ImgInp1, &ImgInp1, BINARY_RGB, INTERMITTENT_PRO1);
		SensorPro(&ImgInp1, &sensor1);
		SensorPro(&ImgInp1, &sensor0);
		break;
#endif

	default:
		break;
	}
}

/************************************************************************/
/**
 * GR-PEACH上 LED制御.
 *
 *	@param[in]		led		点灯RGBパターン
 */
void led_m_rgb(int led)
{
	LED_R = led & 0x1;
	LED_G = (led >> 1) & 0x1;
	LED_B = (led >> 2) & 0x1;
}

/************************************************************************/
/**
 * GR-PEACH上 userボタン.
 *
 *	@return		0:OFF / 1:ON
 */
unsigned char user_button_get(void)
{
	// Read ports with switches
	return (~user_botton) & 0x1; 
}

/************************************************************************/
/**
 * GR-PEACH上 userLED.
 *
 *	@param[in]		led		点灯有無
 */
void led_m_user(int led)
{
	USER_LED = led & 0x01;
}

/************************************************************************/
/**
 * GR-PEACH上 LEDパターン点灯.
 *
 *	@param[in]		set		点灯パターン
 */
void led_m_set(int set)
{
	LedPattern = set;
}

/************************************************************************/
/**
 * LED制御 割り込み用.
 */
void led_m_pro(void)
{
	static long cnt_led_m;
	cnt_led_m++;

	if (pattern <= 0)
	{
		/* NONE */
	}
	else if (pattern <= 20)
	{
		led_m_set(RUN);
	}
	else if (pattern <= 50)
	{
		led_m_set(CRANK);
	}
	else if (pattern <= 70)
	{
		led_m_set(LCHANGE);
	}
	else
	{
		led_m_set(ERROR);
	}

	if (cnt_led_m < led_data[LedPattern][1])
	{
		led_m_rgb(led_data[LedPattern][0]);
	}
	else if (cnt_led_m < (led_data[LedPattern][1] + led_data[LedPattern][2]))
	{
		led_m_rgb(LED_OFF);
	}
	else
	{
		cnt_led_m = 0;
	}
}

/************************************************************************/
/**
 * モータードライブ基板 LED点灯.
 *
 *	@param[in]		led		点灯LED状態
 */
void led_out(int led)
{
	led = ~led;
	LED_3 = led & 0x1;
	LED_2 = (led >> 1) & 0x1;
}

/************************************************************************/
/**
 * モータードライブ基板 SW状態取得.
 *
 *	@return		0:OFF / 1:ON
 */
unsigned char pushsw_get(void)
{
	// Read ports with switches
	return (~push_sw) & 0x1;
}

/************************************************************************/
/**
 * モーターコントロール.
 *
 *	@param[in]		accele_l		左モーターPWM(-100 ～ 100)
 *	@param[in]		accele_r		右モーターPWM(-100 ～ 100)
 */
void motor(int accele_l, int accele_r)
{
	int sw_data = dipsw_get() + 5;

	if (accele_l > 100)
	{
		accele_l = 100;
	}
	else if (accele_l < -100)
	{
		accele_l = -100;
	}
	if (accele_r > 100)
	{
		accele_r = 100;
	}
	else if (accele_r < -100)
	{
		accele_r = -100;
	}

	// ログ保存用 
	motor_buff_l = accele_l;
	motor_buff_r = accele_r;

	// モーター止めモード
	if (~dipsw_get() & 0x08)
	{
		accele_l = 0;
		accele_r = 0;
	}
	else
	{
		accele_l = (accele_l * sw_data) / 20;
		accele_r = (accele_r * sw_data) / 20;
	}

	// 左モーター
	if (accele_l >= 0)
	{
		// 正回転
		Left_motor_signal = 0;
		MTU2TGRC_4 = (long)(MOTOR_PWM_CYCLE - 1) * accele_l / 100;
	}
	else
	{
		// 逆回転
		Left_motor_signal = 1;
		MTU2TGRC_4 = (long)(MOTOR_PWM_CYCLE - 1) * (-accele_l) / 100;
	}

	// 右モーター
	if (accele_r >= 0)
	{
		// 正回転
		Right_motor_signal = 0;
		MTU2TGRD_4 = (long)(MOTOR_PWM_CYCLE - 1) * accele_r / 100;
	}
	else
	{
		// 逆回転
		Right_motor_signal = 1;
		MTU2TGRD_4 = (long)(MOTOR_PWM_CYCLE - 1) * (-accele_r) / 100;
	}
}

/************************************************************************/
/**
 * ステアサーボ制御.
 *
 *	@param[in]		angle		制御角度
 */
void handle(int angle)
{
	// サーボが左から右に逆に動く場合は、「-」を「+」に置き換え
	MTU2TGRD_0 = SERVO_CENTER - angle * HANDLE_STEP;

	angle_buff = angle;
}

/************************************************************************/
/**
 * ディップスイッチ状態取得.
 *
 *	@return		ディップスイッチ状態
 */
unsigned char dipsw_get(void)
{
	return (dipsw.read() & 0x0f);
}

/************************************************************************/
/**
 * 二値化センサ初期化.
 *
 *	@param[in]		ImData		撮影データ
 *	@param[out]		S			二値化データ格納先
 *	@param[in]		PL			ピクセルライン
 *
 *	@return			ErrorCode
 */
int initSensor(mcrMat *ImData, SENSOR *S, int PL)
{
	volatile int X, Y, L;
	volatile int CX;
	volatile int Gap, offset;
	volatile char ErrorCode;

	// 初期化
	offset = 0;
	ErrorCode = 0;

	// 真ん中X座標
	// 左側
	// 画像データの高さ半分の位置に設定
	S->y = (ImData->h - 1);
	L = (ImData->h - 1) * ImData->w;
	if (ImData->bin[L + 0] == 1)
	{
		S->x[5] = 0;
	}
	if (ImData->bin[L + 1] == 1)
	{
		S->x[5] = 1;
	}
	if (ImData->bin[L + 0] == 0 && ImData->bin[L + 1] == 0)
	{
		for (X = 2; X < (ImData->w - 1); X++)
		{
			L = ((ImData->h - 1) * ImData->w) + X;
			if (ImData->bin[L - 2] == 0 &&
				ImData->bin[L - 1] == 0 &&
				ImData->bin[L - 0] == 1 &&
				ImData->bin[L + 1] == 1)
			{
				S->x[5] = X;
				break;
			}
		}
	}
	// 右側
	L = (ImData->h - 1) * ImData->w;
	if (ImData->bin[L + S->x[5] + 0] == 1)
	{
		S->x[2] = S->x[5] + 0;
	}
	if (ImData->bin[L + S->x[5] + 1] == 1)
	{
		S->x[2] = S->x[5] + 1;
	}
	if (ImData->bin[L + S->x[5] + 0] == 1 && ImData->bin[L + S->x[5] + 1] == 1)
	{
		for (X = (S->x[5] + 1); X < (ImData->w - 3); X++)
		{
			L = ((ImData->h - 1) * ImData->w) + X;
			if (ImData->bin[L - 1] == 1 &&
				ImData->bin[L + 0] == 1 &&
				ImData->bin[L + 1] == 0 &&
				ImData->bin[L + 2] == 0)
			{
				S->x[2] = X;
				break;
			}
		}
	}
	CX = S->x[5] + ((S->x[2] - S->x[5]) >> 1);

	// コースの真ん中に車体がない
	if (CX < ((ImData->w >> 1) - 5) || ((ImData->w >> 1) + 5) < CX)
	{
		ErrorCode |= 0x01;
	}

	// トレースライン特定
	for (Y = (ImData->h - 1); Y >= (PL - 1); Y--)
	{
		// 左側
		L = Y * ImData->w;
		for (X = CX; X > 1; X--)
		{
			if (ImData->bin[L + X - 0] == 0 &&
				ImData->bin[L + X + 1] == 0 &&
				ImData->bin[L + X + 2] == 1 &&
				ImData->bin[L + X + 3] == 1)
			{
				S->x[5] = X;
				break;
			}
		}
		// 右側
		for (X = CX; X < (ImData->h - 2); X++)
		{
			if (ImData->bin[L + X - 3] == 1 &&
				ImData->bin[L + X - 2] == 1 &&
				ImData->bin[L + X - 1] == 0 &&
				ImData->bin[L + X + 0] == 0)
			{
				S->x[2] = X;
				break;
			}
		}
		CX = S->x[5] + ((S->x[2] - S->x[5]) >> 1);
		S->y = Y;

		L = ((Y - 1) * ImData->w) + CX;
		if (ImData->bin[L] == 0)
		{
			ErrorCode |= 0x02;
			break;
		}
	}

	// エラーコード確認
	if (!ErrorCode)
	{
		// 真ん中特定
		Gap = ((S->x[2] - S->x[5]) / 3) + offset;
		S->x[4] = S->x[5] + Gap;
		S->x[3] = S->x[2] - Gap;
		// 左側
		S->x[6] = S->x[5] - Gap;
		S->x[7] = S->x[6] - Gap;
		// 左側にセンターライン感知
		if (S->x[5] < 0 || S->x[6] < 0 || S->x[7] < 0)
		{
			ErrorCode |= 0x04;
		}
		// 右側
		S->x[1] = S->x[2] + Gap;
		S->x[0] = S->x[1] + Gap;
		// 右側にセンターライン感知
		if (S->x[2] > (ImData->w - 1) || S->x[1] > (ImData->w - 1) || S->x[0] > (ImData->w - 1))
		{
			ErrorCode |= 0x08;
		}
	}
	return ErrorCode;
}

/************************************************************************/
/**
 * センサ二値化処理.
 *
 *	@param[in]		ImData		撮影データ
 *	@param[out]		S			二値化データ格納先
 */
void SensorPro(mcrMat *ImData, SENSOR *S)
{
#if 0
	long L;
	unsigned char sensor = 0;

	sensor = 0;
	for (uint8_t i = 0; i < 8; i++)
	{
		L = (S->y * ImData->w) + S->x[i];
		sensor |= (ImData->bin[L] & 0x01) << i;
		ImData->r[L] = 255;
		ImData->g[L] = 0;
		ImData->b[L] = 0;
	}
	S->v = sensor & 0xff;
#else

	char data[8] = {};
	char max = 0, min = 255;

	for (uint8_t i = 0; i < 8; i++)
	{
		data[i] = getShrinkImage(S->x[i], S->y);
		if (max <= data[i])
		{
			// 8個のうち、最大を見つける
			max = data[i];
		}
		if (min >= data[i])
		{
			// 8個のうち、最小を見つける
			min = data[i];
		}
	}

	// 隣同士の差の絶対値
	int sa_7_6 = abs(data[7] - data[6]);
	int sa_6_5 = abs(data[6] - data[5]);
	int sa_5_4 = abs(data[5] - data[4]);
	int sa_4_3 = abs(data[4] - data[3]);
	int sa_3_2 = abs(data[3] - data[2]);
	int sa_2_1 = abs(data[2] - data[1]);
	int sa_1_0 = abs(data[1] - data[0]);

	int shiki = 0;
	if (max >= THRESHOLD)
	{
		// 最大値がs以上なら、sをしきい値とする
		shiki = THRESHOLD;
	}
	else if (sa_7_6 >= 8 || sa_6_5 >= 8 || sa_5_4 >= 8 ||
			 sa_4_3 >= 8 || sa_3_2 >= 8 || sa_2_1 >= 8 || sa_1_0 >= 8)
	{
		// 隣同士の差が１つでも「差」以上なら、８点の（最大値－最小値）×0.7 + 最小値　をしきい値とする
		shiki = (max - min) * 7 / 10 + min;
	}
	else
	{
		// 当てはまらなければ、しきい値を256とする、すなわちすべて0となる
		shiki = 256;
	}

	// d[7]～d[0]をbit7～bit0に割り当てる
	S->v = 0;
	for (uint8_t i = 0; i < 8; i++)
	{
		S->v <<= 1;
		S->v |= (data[i] >= shiki ? 1 : 0);
	}
#endif
}

/************************************************************************/
/**
 * 二値化センサ状態取得.
 *
 *	@param[in]		S			二値化データ
 *	@param[in]		mask		マスク値
 *
 *	@return			センサ状態
 */
unsigned char sensor_inp(SENSOR *S, unsigned char mask)
{
	return (S->v & 0xff) & mask;
}

/************************************************************************/
/**
 * スタートバー判定.
 *
 *	@return		ON：1 / OFF：0
 */
int startbar_get(void)
{
	unsigned char b;
	int ret;

	ret = 0;
	// Read start bar signal
	b = sensor_inp(&sensor1, MASK3_3); 
	if (b == 0xe7)
	{
		ret = 1;
	}

	return ret;
}

/************************************************************************/
/**
 * クロスライン判定.
 *
 *	@return		ON：1 / OFF：0
 */
int check_crossline(void)
{
	unsigned char b;
	int ret;

	ret = 0;
	b = sensor_inp(&sensor0, MASK3_3);
	if (b == 0xe7)
	{
		ret = 1;
	}
	return ret;
}

/************************************************************************/
/**
 * 右ハーフライン判定.
 *
 *	@return		ON：1 / OFF：0
 */
int check_rightline(void)
{
	unsigned char b;
	int ret;

	ret = 0;
	b = sensor_inp(&sensor0, MASK4_4);
	if (b == 0x1f)
	{
		ret = 1;
	}
	return ret;
}

/************************************************************************/
/**
 * 左ハーフライン判定.
 *
 *	@return		ON：1 / OFF：0
 */
int check_leftline(void)
{
	unsigned char b;
	int ret;

	ret = 0;
	b = sensor_inp(&sensor0, MASK4_4);
	if (b == 0xf8)
	{
		ret = 1;
	}
	return ret;
}

/************************************************************************/
/**
 * センサ情報シリアル出力.
 * 
 *	@param[in]		ImData		表示カメラ値
 *	@param[in]		mode		0:FULL / 1:TOP / 2:BOTTOM
 */
void SerialOut_TeraTerm(mcrMat *ImData, int Mode)
{
	volatile long X, Y, H;
	volatile int i;

	switch (Mode)
	{
	case 0:
		// FULL
		H = ImData->h;
		for (Y = 0; Y < ImData->h; Y++)
		{
			for (X = 0; X < ImData->w; X++)
			{
				pc.printf("%d ", ImData->bin[(Y * ImData->w) + X]);
			}
			if (Y == sensor1.y)
				pc.printf("%2d sensor1 ", sensor1.y);
			if (Y == sensor0.y)
				pc.printf("%2d sensor0 ", sensor0.y);
			pc.printf("\n\r");
		}
		break;
	case 1:
		// TOP
		H = ImData->h >> 1;
		for (Y = 0; Y < (ImData->h >> 1); Y++)
		{
			for (X = 0; X < ImData->w; X++)
			{
				pc.printf("%d", ImData->bin[(Y * ImData->w) + X]);
			}
			if (Y == sensor1.y)
				pc.printf("%2d sensor1 ", sensor1.y);
			if (Y == sensor0.y)
				pc.printf("%2d sensor0 ", sensor0.y);
			pc.printf("\n\r");
		}
		break;
	case 2:
		// BOTTOM
		H = ImData->h >> 1;
		for (Y = (ImData->h >> 1); Y < ImData->h; Y++)
		{
			for (X = 0; X < ImData->w; X++)
			{
				pc.printf("%d", ImData->bin[(Y * ImData->w) + X]);
			}
			if (Y == sensor1.y)
				pc.printf("%2d sensor1 ", sensor1.y);
			if (Y == sensor0.y)
				pc.printf("%2d sensor0 ", sensor0.y);
			pc.printf("\n\r");
		}
		break;
	default:
		break;
	}

	// Add display( sensor )
	pc.printf("\n\r");
	for (Y = sensor1.y, X = 0, i = 7; i >= 0; i--)
	{
		for (; X < sensor1.x[i]; X++)
			pc.printf("  ");
		pc.printf("%1d ", ImData->bin[(Y * ImData->w) + X]);
		X++;
	}
	pc.printf("\n\r");
	for (Y = sensor0.y, X = 0, i = 7; i >= 0; i--)
	{
		for (; X < sensor0.x[i]; X++)
			pc.printf("  ");
		pc.printf("%1d ", ImData->bin[(Y * ImData->w) + X]);
		X++;
	}
	pc.printf("\n\r");
	H += 3;

	// Add display
	pc.printf("\n\r");
	pc.printf("ThresholdBuff = %3d, Min = %3d, Max = %3d\n\r", ThresholdBuff, ThresholdMin, ThresholdMax);
	pc.printf("startbar_get  = %1d \n\r", startbar_get());
	pc.printf("sensor_inp    = 0x%02x \n\r", sensor_inp(&sensor0, 0xff));
	pc.printf("\n\r");
	H += 5;

	pc.printf("\033[%dA", H);
}

//------------------------------------------------------------------//
// Serial Out ( Excel )
// ImData     : mcrMat
// Mode       : 0 -> Y data
//              1 -> R data
//              2 -> G data
//              3 -> B data
//              4 -> H data
//              5 -> S data
//              6 -> L data
//------------------------------------------------------------------//
void SerialOut_Excel(mcrMat *ImData, int Mode)
{
	volatile long X, Y;

	for (Y = 0; Y < ImData->h; Y++)
	{
		for (X = 0; X < ImData->w; X++)
		{
			if (Mode == 0)
				pc.printf("%3d,", ImData->y[(Y * ImData->w) + X]);
			else if (Mode == 1)
				pc.printf("%3d,", ImData->r[(Y * ImData->w) + X]);
			else if (Mode == 2)
				pc.printf("%3d,", ImData->g[(Y * ImData->w) + X]);
			else if (Mode == 3)
				pc.printf("%3d,", ImData->b[(Y * ImData->w) + X]);
			else if (Mode == 4)
				pc.printf("%3d,", ImData->H[(Y * ImData->w) + X]);
			else if (Mode == 5)
				pc.printf("%3d,", ImData->S[(Y * ImData->w) + X]);
			else if (Mode == 6)
				pc.printf("%3d,", ImData->L[(Y * ImData->w) + X]);
		}
		pc.printf("\n\r");
	}
}

//------------------------------------------------------------------//
// Serial Out ( csv -> jpg )
// ImData      : mcrMat
// Format      : VIDEO_YCBCR422 or VIDEO_RGB888
// ColorPattern: COLOR
//               BINARY
//               COLOR_BINARY
//------------------------------------------------------------------//
void SerialOut_CsvJpg(mcrMat *ImData, int Format, int ColorPattern)
{
	volatile long X, Y, L, D;

	pc.printf("//,X-Size,Y-Size");
	pc.printf("\n\r");
	pc.printf("#SIZE,%3d,%3d", ImData->w, ImData->h);
	pc.printf("\n\r");
	if (Format == VIDEO_YCBCR422)
		pc.printf("//,X-Point,Y-Point,Y,CB,CR");
	else if (Format == VIDEO_RGB888)
		pc.printf("//,X-Point,Y-Point,R,G,B");
	pc.printf("\n\r");

	switch (ColorPattern)
	{
	case COLOR:
		for (Y = 0; Y < ImData->h; Y++)
		{
			L = ImData->w * Y;
			for (X = 0; X < ImData->w; X++)
			{
				D = L + X;
				if (Format == VIDEO_YCBCR422)
					pc.printf("#YCbCr,");
				else if (Format == VIDEO_RGB888)
					pc.printf("#RGB,");
				pc.printf("%3d,", X);
				pc.printf("%3d,", Y);
				if (Format == VIDEO_YCBCR422)
				{
					pc.printf("%3d,", ImData->y[D]);
					pc.printf("%3d,", ImData->cb[D]);
					pc.printf("%3d,", ImData->cr[D]);
				}
				else if (Format == VIDEO_RGB888)
				{
					pc.printf("%3d,", ImData->r[D]);
					pc.printf("%3d,", ImData->g[D]);
					pc.printf("%3d,", ImData->b[D]);
				}
				pc.printf("\n\r");
			}
		}
		break;

	case BINARY:
		for (Y = 0; Y < ImData->h; Y++)
		{
			L = ImData->w * Y;
			for (X = 0; X < ImData->w; X++)
			{
				D = L + X;
				if (Format == VIDEO_YCBCR422)
					pc.printf("#YCbCr,");
				else if (Format == VIDEO_RGB888)
					pc.printf("#RGB,");
				pc.printf("%3d,", X);
				pc.printf("%3d,", Y);
				if (Format == VIDEO_YCBCR422)
				{
					if (ImData->bin[D])
					{
						pc.printf("%3d,", 255);
						pc.printf("%3d,", 128);
						pc.printf("%3d,", 128);
					}
					else
					{
						pc.printf("%3d,", 0);
						pc.printf("%3d,", 128);
						pc.printf("%3d,", 128);
					}
				}
				else if (Format == VIDEO_RGB888)
				{
					if (ImData->bin[D])
					{
						pc.printf("%3d,", 255);
						pc.printf("%3d,", 255);
						pc.printf("%3d,", 255);
					}
					else
					{
						pc.printf("%3d,", 0);
						pc.printf("%3d,", 0);
						pc.printf("%3d,", 0);
					}
				}
				pc.printf("\n\r");
			}
		}
		break;

	case COLOR_BINARY:
		for (Y = 0; Y < ImData->h; Y++)
		{
			L = ImData->w * Y;
			for (X = 0; X < ImData->w; X++)
			{
				D = L + X;
				if (Format == VIDEO_YCBCR422)
					pc.printf("#YCbCr,");
				else if (Format == VIDEO_RGB888)
					pc.printf("#RGB,");
				pc.printf("%3d,", X);
				pc.printf("%3d,", Y);
				if (Format == VIDEO_YCBCR422)
				{
					if (ImData->bin[D])
					{
						pc.printf("%3d,", ImData->y[D]);
						pc.printf("%3d,", ImData->cb[D]);
						pc.printf("%3d,", ImData->cr[D]);
					}
					else
					{
						pc.printf("%3d,", 0);
						pc.printf("%3d,", 128);
						pc.printf("%3d,", 128);
					}
				}
				else if (Format == VIDEO_RGB888)
				{
					if (ImData->bin[D])
					{
						pc.printf("%3d,", ImData->r[D]);
						pc.printf("%3d,", ImData->g[D]);
						pc.printf("%3d,", ImData->b[D]);
					}
					else
					{
						pc.printf("%3d,", 0);
						pc.printf("%3d,", 0);
						pc.printf("%3d,", 0);
					}
				}
				pc.printf("\n\r");
			}
		}
		break;

	default:
		break;
	}
	pc.printf("End\n\r");
}

//------------------------------------------------------------------//
// Debug Process Functions
//------------------------------------------------------------------//
void DebugPro(void)
{
	volatile int Number; /* Serial Debug Mode only   */

	pc.printf("Serial Debug Mode \n\r");
	pc.printf("\n\r");
#if (VIDEO_FORMAT == VIDEO_YCBCR422)
	pc.printf("VIDEO_FORMAT = VIDEO_YCBCR422 \n\r");
#endif
#if (VIDEO_FORMAT == VIDEO_RGB888)
	pc.printf("VIDEO_FORMAT = VIDEO_RGB888 \n\r");
#endif
	pc.printf("\n\r");
	pc.printf("0:TeraTram Real-time display (Binary)\n\r");
	pc.printf("1:TeraTram Real-time display (convert 0 or 1)\n\r");
	pc.printf("2:TeraTram Real-time display (detection center line))\n\r");
	pc.printf("3:Excel(csv) (Y)(R)(G)(B)\n\r");
	pc.printf("4:Excel(csv) -> csv_jpg_convert.bat (Color)\n\r");
	pc.printf("5:Excel(csv) -> csv_jpg_convert.bat (Binary)\n\r");
	pc.printf("6:Excel(csv) -> csv_jpg_convert.bat (Color - Binary)\n\r");
	pc.printf("7:trace pattern debug\n\r");
	pc.printf("\n\r");
	pc.printf("Please Number\n\r");
	pc.printf("No = ");
	do {
		pc.scanf("%d", &Number);
	} while(Number > 7);
	pc.printf("\n\r");
	pc.printf("Please push the SW ( on the Motor drive board )\n\r");
	pc.printf("\n\r");
	// while (!pushsw_get());
	while (PNumber <= 15);

	// 画面クリア
	pc.printf("\033[H");
	pc.printf("\033[2J");
	// 圧縮データを保存
	ImgBuff = ImgInp1;

	switch (Number)
	{
	case 0:
		// TeraTram Real-time display (Binary)
		while (1)
		{
			SerialOut_TeraTerm(&ImgInp1, 0);
			pc.printf( "\033[H" );
		}
		break;
	case 1:
		while (1)
		{
			// 補正値で表示 しきい値180以上を"1" 180を変えると、しきい値が変わる
			pc.printf("shikii chi 180\r\n");
			for (uint y = 0; y < 30; y++)
			{
				pc.printf( "%3d:%08ld ", y +  0, convertCharToLong(convertBinarization(y +  0, 180, 8)));
				pc.printf( "%3d:%08ld ", y + 30, convertCharToLong(convertBinarization(y + 30, 180, 8)));
				pc.printf( "%3d:%08ld ", y + 60, convertCharToLong(convertBinarization(y + 60, 180, 8)));
				pc.printf( "%3d:%08ld ", y + 90, convertCharToLong(convertBinarization(y + 90, 180, 8)));
				pc.printf( "\r\n");
			}
			pc.printf( "\033[H" );
		}
		break;
	case 2:
		while (1)
		{
			// https://www.sejuku.net/blog/24934
			// 文字の色
			// \x1b[30m 黒 \x1b[31m 赤 \x1b[32m 緑 \x1b[33m 黄
			// \x1b[34m 青 \x1b[35m マゼンダ \x1b[36m シアン
			// \x1b[37m 白 \x1b[39m デフォルトに戻す
			//背景色の指定
			// \x1b[40m 黒 \x1b[41m 赤 \x1b[42m 緑 \x1b[43m 黄
			// \x1b[44m 青 \x1b[45m マゼンダ \x1b[46m シアン
			// \x1b[47m 灰 \x1b[49m デフォルトに戻す
			// 60～119行を表示(しきい値120以上を"1"とする)
			pc.printf( "    0         1         2         3         4         5         6         7         8\r\n");
			pc.printf( "120 012345678901234567890123456789012345678901234567890123456789012345678901234567890\r\n");
			for (uint8_t y = 0; y < imgSizeY; y += 2)
			{
				pc.printf("%03d:", y );
				for (uint8_t x = 0; x < imgSizeX; x++)
				{
					bool isWriteFlag = true;
					// 180を変えるとしきい値が変わる
					uint8_t c = getShrinkImage(x, y) >= 200 ? 1 : 0;
					for (uint8_t i = 0; i < imgSizeX; i++)
					{
						if (x == topDataLeftToRight[y][i] && isWriteFlag == true)
						{
							pc.printf( "\x1b[44m%d\x1b[49m", c );
							isWriteFlag = false;
						}
						if (x == topDataRightToLeft[y][i] && isWriteFlag == true)
						{
							pc.printf( "\x1b[41m%d\x1b[49m", c );
							isWriteFlag = false;
						}
						if (x == centerPoint[y] && isWriteFlag == true)
						{
							pc.printf( "\x1b[45m%d\x1b[49m", c );
							isWriteFlag = false;
						}
						else if (isWriteFlag == true && i == 29)
						{
							pc.printf( "%d", c );
							isWriteFlag = false;
						}
					}
					if (x == (imgSizeX - 1))
					{
#if 1
						pc.printf( " topDif%4d th%4d T%4d camera_D%4d line_P %2d center %2d ltr %2d rtl %2d",
									lineData[y].topDif,
									lineData[y].th,
									lineData[y].topCamera,
									lineData[y].cameraTh,
									centerPoint[y],
									lineData[y].centerCnt,
									lineData[y].topCnt_LTR,
									lineData[y].topCnt_RTL);
						pc.printf(" line_find ");
						for (uint8_t cnt = 0; cnt < lineData[y].centerCnt; cnt++)
						{
							pc.printf("%2d / ", lineCData[y][cnt].Point);
						}
#endif
					}
				}
				pc.printf( "  \r\n" );
			}
#if 1
			pc.printf("handle %3d\r\n", handleVal);
			pc.printf("sensor1 val:0x%02X / ", sensor1.v);
			pc.printf("sensor0 val:0x%02X\r\n", sensor0.v);
			pc.printf("line left %d / ", isHalfLine_L);
			pc.printf("line right %d / ", isHalfLine_R);
			pc.printf("cross line %d\r\n", isCrossLine);
			pc.printf("Encoder %03d\r\n", encoderInfo.totalCount);
			pc.printf("Pattern %03d\r\n", pattern);
			pc.printf("DiffCenter %03d\r\n", centerDiffAve);
		//	pc.printf( "%3d,%3d,\r\n", p_value, i_value, d_value);
			autoRun(handleVal, 0);
#endif
			// 画面クリア
			pc.printf( "\033[H" );
		}
		break;
	case 3:
		// Excel(csv)
		pc.printf("Excel(csv)(Y)\n\r");
		pc.printf("\n\r");
		SerialOut_Excel(&ImgBuff, 0);
		pc.printf("\n\r");

		pc.printf("Excel(csv)(R)\n\r");
		pc.printf("\n\r");
		SerialOut_Excel(&ImgBuff, 1);
		pc.printf("\n\r");

		pc.printf("Excel(csv)(G)\n\r");
		pc.printf("\n\r");
		SerialOut_Excel(&ImgBuff, 2);
		pc.printf("\n\r");

		pc.printf("Excel(csv)(B)\n\r");
		pc.printf("\n\r");
		SerialOut_Excel(&ImgBuff, 3);
		pc.printf("\n\r");
		while (1);
		break;
	case 4:
		// Excel(csv) -> csv_jpg_convert.bat (Color)
		SerialOut_CsvJpg(&ImgBuff, VIDEO_FORMAT, COLOR);
		while (1);
		break;
	case 5:
		// Excel(csv) -> csv_jpg_convert.bat (Binary)
		SerialOut_CsvJpg(&ImgBuff, VIDEO_FORMAT, BINARY);
		while (1);
		break;
	case 6:
		// Excel(csv) -> csv_jpg_convert.bat (Color Binary)
		SerialOut_CsvJpg(&ImgBuff, VIDEO_FORMAT, COLOR_BINARY);
		while (1);
		break;
	default:
		break;
	}
}

/************************************************************************/
/**
 * 撮影画素値取得.
 * 
 *	@param[in]		x		取得したいX座標
 *	@param[in]		y		取得したいY座標
 *
 *	@return			画素値
 */
inline char getImage(int x, int y)
{
	return ImgInp0.y[x + PIXEL_HW * y];
}

/************************************************************************/
/**
 * 圧縮画像画素値取得.
 * 
 *	@param[in]		x		取得したいX座標
 *	@param[in]		y		取得したいY座標
 *
 *	@return			画素値
 */
inline char getShrinkImage(int x, int y)
{
	return ImgInp1.y[x + (PIXEL_HW / 2) * y];
}

/************************************************************************/
/**
 * 圧縮画像サイズ取得.
 * 
 *	@param[out]		x		横画素数
 *	@param[out]		y		縦画素数
 */
inline void getShrinkImageSize(uint16_t &x, uint16_t &y)
{
	if (ImgInp1.w && ImgInp1.h)
	{
		x = ImgInp1.w;
		y = ImgInp1.h;
	}
	else
	{
		x = CAMERA_LINE_X;
		y = CAMERA_LINE_Y;
	}
}

/************************************************************************/
/**
 * 圧縮画像画素値の微分値を計算.
 */
void setDiffData(void)
{
	// 圧縮したデータをカメラデータにセットする
	for (uint8_t y = 0; y < imgSizeY; y++)
	{
		// データコピー
		for (uint8_t x = 0; x < imgSizeX; x++)
		{
			diffData[y][x].camera = getShrinkImage(x, y);
		}
		// 最後のデータを含めない(0～77)
		for (uint8_t x = 0; x < imgSizeX - 2; x++)
		{
			// 左から右引いた差
			diffData[y][x].dif_LTR = diffData[y][x].camera - diffData[y][x + 1].camera;
		}
		// 最初のデータを含めない(2～79)
		for (int x = imgSizeX - 1; x > 1; x--)
		{
			// 右から左引いた差
			diffData[y][x].dif_RTL = diffData[y][x].camera - diffData[y][x - 1].camera;
		}
	}
}

/************************************************************************/
/**
 * センターライン検出.
 */
void getCenterLine(void)
{
	//case -2//検索用
	// 閾値作成
	getThreshold();
	// 境目検出
	findBorder();
	// 境目と境目の関連付け
	topAssociation();
	// センター候補検出
	getCenterCandidate();
	// 各行のセンターを算出
	getCenterNumber();
}

/************************************************************************/
/**
 * ライン検出閾値作成.
 */
void getThreshold(void)
{
	int averageCnt = 0;
	float averageVal = 0;

	// ハーフラン認識中は閾値を更新しない
	if (!isHalfLine)
	{
		for (int8_t y = imgSizeY - 1; y >= 0; y--)
		{
			lineData[y].topDif = 0;
			lineData[y].topCamera = 0;

			uint8_t countX = imgSizeX;
			DIFF_DATA* data = diffData[y];

			do {
				// 差の最大値取得
				if (lineData[y].topDif <= data->dif_LTR)
				{
					lineData[y].topDif = data->dif_LTR;
				}
				// 差の最大値取得
				if (lineData[y].topDif <= data->dif_RTL)
				{
					lineData[y].topDif = data->dif_RTL;
				}
				// カメラの最大値取得
				if (lineData[y].topCamera <= data->camera)
				{
					lineData[y].topCamera = data->camera;
					averageVal += data->camera;
					averageCnt += 1;
				}
				data++;
			} while(countX--);

#if 0	// 旧プログラム
			for (uint8_t x = 0; x < imgSizeX; x++)
			{
				// 差の最大値取得
				if (lineData[y].topDif <= diffData[y][x].dif_LTR)
				{
					lineData[y].topDif = diffData[y][x].dif_LTR;
				}
				// 差の最大値取得
				if (lineData[y].topDif <= diffData[y][x].dif_RTL)
				{
					lineData[y].topDif = diffData[y][x].dif_RTL;
				}
				// カメラの最大値取得
				if (lineData[y].topCamera <= diffData[y][x].camera)
				{
					lineData[y].topCamera = diffData[y][x].camera;
					averageVal += diffData[y][x].camera;
					averageCnt += 1;
				}
			}
#endif

			// 最大値×定数で閾値に
			lineData[y].th = lineData[y].topDif * THGAIN;
			// この二つの閾値の一体化検討
			lineData[y].cameraTh = lineData[y].topCamera * 0.6;
			if (!isHalfLine)
			{
				HL_Data[y].Th = lineData[y].topCamera * HLGAIN;
			}

			// 保険
			if (lineData[y].th <= 10)
			{
				lineData[y].th = 10;
			}
			if (lineData[y].cameraTh <= 100)
			{
				lineData[y].cameraTh = 100;
			}
			if (HL_Data[y].Th <= 100)
			{
				HL_Data[y].Th = 100;
			}
		}

		// カメラの最大値計算
		averageVal = averageVal / averageCnt;
		difCameraAverage = lastCameraAverage - averageVal;
		lastCameraAverage = averageVal;

		if (fabsf(difCameraAverage) >= 20)
		{
			isSlopeUnder = true;
			cnt_slope_under = 0;
		}
		if (isSlopeUnder && cnt_slope_under >= 50)
		{
			isSlopeUnder = false;
			cnt_slope_under = 0;
		}

		if (difCameraAverage_max <= difCameraAverage)
		{
			difCameraAverage_max = difCameraAverage;
		}
	}
}

/************************************************************************/
/**
 * 白と黒の境界を検出.
 */
void findBorder(void)
{
	bool isActive[imgSizeY][2] = {false};
	// 左端の座標
	int fastLine = 1;
	// 右端の座標
	int lastLine = imgSizeX - 2;

	for (uint8_t y = 0; y < imgSizeY; y++)
	{
		uint16_t cntLeftToRight = 0;
		uint16_t cntRightToLeft = 0;

		// 初期化
		for (uint8_t x = 0; x < CAMERA_LINE_X; x++)
		{
			// 減算値の初期化
			topDataRightToLeft[y][x] = -1;
			topDataLeftToRight[y][x] = -1;
		}

		// 右から順にチェック(1～77)
		for (uint8_t x = imgSizeX - 3; x > 0; x--)
		{
			// 差を見る
			if (diffData[y][x].dif_LTR >= lineData[y].th && !isActive[y][0])
			{
				isActive[y][0] = true;
				topDataLeftToRight[y][cntLeftToRight++] = x;
			}
			else if (diffData[y][x].dif_LTR <= -(lineData[y].th * 2) && isActive[y][0])
			{
				// 連続で記録しないように白から黒に戻るまで待機
				isActive[y][0] = false;
			}
		}

		// 左から順にチェック(2～78)
		for (uint8_t x = 2; x < imgSizeX - 1; x++)
		{
			if (diffData[y][x].dif_RTL >= lineData[y].th && !isActive[y][1])
			{
				isActive[y][1] = true;
				topDataRightToLeft[y][cntRightToLeft++] = x;
			}
			else if (diffData[y][x].dif_RTL <= -(lineData[y].th * 2 ) && isActive[y][1])
			{
				isActive[y][1] = false;
			}
		}

		// 画面の端の処理(画像が白から始まると差が発生していないので検知できない)
		if (lineData[y].cameraTh <= diffData[y][lastLine].camera)
		{
			// 右端が白だったとき
			topDataLeftToRight[y][cntLeftToRight++] = lastLine;
		}
		if (lineData[y].cameraTh <= diffData[y][fastLine].camera)
		{
			// 左端が白だった時
			topDataRightToLeft[y][cntRightToLeft++] = fastLine;
		}

		lineData[y].WHTcnt = 0;
		// simpleLineCheckで使う
		for (uint8_t x = 0; x < imgSizeX; x++)
		{
			if (diffData[y][x].camera >= lineData[y].cameraTh)
			{
				lineData[y].WHTcnt++;
			}
		}
		lineData[y].topCnt_LTR = cntLeftToRight;
		lineData[y].topCnt_RTL = cntRightToLeft;
	}
}

/************************************************************************/
/**
 * 境目と境目の関連付け.
 */
void topAssociation(void)
{
	// pointとpointの最大許容差
	int maxDif = 0;
	// 最低許容差
	int minDif = 0;

	for (uint8_t y = 0; y < imgSizeY; y++)
	{
		uint16_t cntCenter = 0;
		for (uint8_t x = 0; x < CAMERA_LINE_X; x++)
		{
			lineCData[y][x].topPoint_LTR = -1;
			lineCData[y][x].topPoint_RTL = -1;
		}

		// Google関数グラフで作った式(yの関数式)12(今はほぼ意味がないこれから坂検知で多分使う)
		maxDif = (0.31f * y) + 17;
		// Google関数グラフで作った式(yの関数式)1
		minDif = (0.31f * y) + 1;
		// point同士の全ての組み合わせを試行
		for (int i_LTR = 0; i_LTR <= lineData[y].topCnt_LTR; i_LTR++)
		{
			for (int i_RTL = 0; i_RTL <= lineData[y].topCnt_RTL; i_RTL++)
			{
				// LTRの左側にRTLがないと検知しない
				if (topDataRightToLeft[y][i_RTL] <= topDataLeftToRight[y][i_LTR])
				{
					// 差が一定以上かつ一定以下なら認識
					if (checkAssociation(y, maxDif, minDif, i_LTR, i_RTL))
					{
						lineCData[y][cntCenter].topPoint_LTR = topDataLeftToRight[y][i_LTR];
						lineCData[y][cntCenter].topPoint_RTL = topDataRightToLeft[y][i_RTL];
						cntCenter++;
					}
				}
			}
		}
		lineData[y].centerCnt = cntCenter;
	}
}

/************************************************************************/
/**
 * センター候補取得.
 */
void getCenterCandidate(void)
{
	// 二つのtopDataの中間がセンターポイント
	for (uint8_t y = 0; y < imgSizeY; y++)
	{
		for (uint8_t x = 0; x < CAMERA_LINE_X; x++)
		{
			lineCData[y][x].Point = -1;
		}
		for (int i = 0; i <= lineData[y].centerCnt; i++)
		{
			lineCData[y][i].Point = lineCData[y][i].topPoint_LTR - lineCData[y][i].topPoint_RTL;
			lineCData[y][i].Point = lineCData[y][i].Point * 0.5f;
			lineCData[y][i].Point = lineCData[y][i].Point + lineCData[y][i].topPoint_RTL;
			senterPoint_D[y][i]   = lineCData[y][i].Point;//debug
		}
	}
}

/************************************************************************/
/**
 * 各行のセンター検出.
 */
void getCenterNumber(void)
{
	int bottomY = imgSizeY - 1;
	int newPoint = 0;
	int underPoint = 0;
	// 左端
	int fastLine = 1;
	//右端
	int lastLine = imgSizeX - 2;
	// 許容差
	canDif = 12;

	lineData[bottomY].PointDif = imgSizeX;
	lineData[bottomY].No = 0;
	lineData[bottomY].isLineTrue = false;
	// 一番下だけ別処理(前回の値との差)
	for (int i = 0; i <= lineData[bottomY].centerCnt; i++)
	{
		// より前回とのが差小さいものを選ぶ
		if (lineData[bottomY].PointDif >= abs(centerPoint[bottomY] - lineCData[bottomY][i].Point))
		{
			lineData[bottomY].PointDif = abs(centerPoint[bottomY] - lineCData[bottomY][i].Point);
			// 何個目のセンターか記録
			lineData[bottomY].No = i;
		}
	}
	// 候補の値
	newPoint = lineCData[bottomY][lineData[bottomY].No].Point;
	if (canDif * 2 >= abs(centerPoint[bottomY] - newPoint) && !simpleLineCheck(bottomY))
	{
		centerPoint[bottomY] = newPoint;
		isUpdateLine = true;
		lineData[bottomY].isLineTrue = true;
	}
	else
	{
		isUpdateLine = false;
		lineData[bottomY].isLineTrue = false;
	}
	if (centerPoint[bottomY] == -1)
	{
		if (lineData[bottomY].centerCnt == 1)
		{
			centerPoint[bottomY] = lineCData[bottomY][0].Point;
		}
		isUpdateLine = true;
		lineData[bottomY].isLineTrue = true;
	}

	// ここから一番下以外の処理(下の行との差)
	for (int y = imgSizeY - 2; y >= 0; y--)
	{
		lineData[y].PointDif = imgSizeX;
		lineData[y].No = 0;
		lineData[y].isLineTrue = false;
		for (int i = 0; i < lineData[y].centerCnt; i++)
		{
			// 下の行のセンターと最も近いセンターの検出
			underPoint = centerPoint[y + 1];
			if (lineData[y].PointDif >= abs(lineCData[y][i].Point - underPoint))
			{
				lineData[y].PointDif = abs(lineCData[y][i].Point - underPoint);
				lineData[y].No = i;
			}
		}
		// 下のpointと離れすぎていた時の処理
		newPoint = lineCData[y][lineData[y].No].Point;
		// (上で既に実行済み)
		underPoint = centerPoint[y + 1];
		// 許容差を超えたら下のpointを今のpointに上書きする
		if (canDif >= abs(underPoint - newPoint) && !simpleLineCheck(y) &&
			newPoint < lastLine && newPoint > fastLine)
		{
			centerPoint[y] = newPoint;
			lineData[y].isLineTrue = true;
		}
		else
		{
			centerPoint[y] = underPoint;
			lineData[y].isLineTrue = false;
		}
	}
}

/************************************************************************/
/**
 * 差が一定以上かつ一定以下であるか判定.
 * 
 *	@param[in]		y			ライン位置
 *	@param[in]		maxDif		差分最大値
 *	@param[in]		minDif		差分最小値
 *	@param[in]		i_LTR		判定差分値
 *	@param[in]		i_RTL		判定差分値
 *
 *	@return			差が一定以上かつ一定以下である場合true
 */
bool checkAssociation(int y, int maxDif, int minDif, int i_LTR, int i_RTL)
{
	// 左端
	int fastLine = 1;
	// 右端
	int lastLine = imgSizeX - 2;
	if (maxDif >= abs(topDataLeftToRight[y][i_LTR] - topDataRightToLeft[y][i_RTL]))
	{
		if (minDif <= abs(topDataLeftToRight[y][i_LTR] - topDataRightToLeft[y][i_RTL]) ||
		   (fastLine == topDataRightToLeft[y][i_RTL] || lastLine == topDataLeftToRight[y][i_LTR]))
		{
			// ↑pointが画面端に触れていたら条件'一定以上'をパス↑
			return true;
		}
	}
	return false;
}

/************************************************************************/
/**
 * ハンドル値自動生成.
 * 
 *	@param[in]		linePoint		ライン位置
 *
 *	@return			ハンドル値
 */
int makeHandleVal(const int* linePoint)
{
	uint16_t diffAdd = 0;
	uint16_t diffCnt = 0;
	// 見えている範囲のズレの平均をとる
	for (uint8_t y = 20; y < imgSizeY; y++)
	{
		diffAdd += linePoint[y];
	//	pc.printf("%3d / ", centerPoint[y]);
		diffCnt++;
	}
//	pc.printf("%\r\n");
	centerDiffAve = diffAdd / diffCnt;
//	pc.printf("diffAdd %5d %5d\r\n", diffAdd, centerDiffAve);
//	pc.printf( "\033[H" );

	//int raw_handleVal = (cameraCenter + traceLineOffsetVal) - linePoint;
	int raw_handleVal = (cameraCenter + traceLineOffsetVal) - centerDiffAve;
	float dif_handleVal = raw_handleVal - last_raw_handleVal;
#if 1
	int retHandleVal = 0;
	// ハンドルの値生成
	if (TOLERANCE_VAL <= abs(raw_handleVal))
	{
		retHandleVal = (raw_handleVal * HA_P_GAIN) + (dif_handleVal * HA_D_GAIN);
	}
	last_raw_handleVal = retHandleVal;

	return retHandleVal;
#endif

#if 0
	int handleVal = (cameraCenter + traceLineOffsetVal) - linePoint;
	// ハンドルの値生成
	// 画面の中心と線の場所の差を1.6乗してgainをかけて値を作ってる
	if (TOLERANCE_VAL <= abs(handleVal))
	{
		handleVal = handleVal * HAGAIN;
	}
	else
	{
		handleVal = 0;
	}

	return handleVal;
#endif
#if 0
	// gain調整の余地あり
	handleVal = pow(abs(handleVal), HINDEX) * HANDEX_GAIN;
	// マイナスの値を累乗出来ないのであとで符号を付与
	if (0 > (cameraCenter - linePoint))
	{
		return -(handleVal);
	}
	else
	{
		return handleVal;
	}
#endif
}

/************************************************************************/
/**
 * 自動ライントレース.
 * 
 *	@param[in]		handleVal		ハンドル切れ角度
 *	@param[in]		power			モーターパワー
 */
void autoRun(const int handleVal, const int power)
{
	int leftMotorVal, rightMotorVal;
	int setPower = power;

	// 左右のモータの出力を計算
	float MoOutgain =  (MOGAIN_GAIN * abs(handleVal));
	float MoIngain = 1 - MoOutgain;
	MoOutgain += 1;

	// 内側のモーターの値にgainをかける
	if (handleVal >= 0)
	{
		leftMotorVal  = setPower * MoIngain;
		rightMotorVal = setPower * MoOutgain;
	}
	else
	{
		leftMotorVal  = setPower * MoOutgain;
		rightMotorVal = setPower * MoIngain;
	}

	// カーブにいるときの処理
	if (abs(handleVal) >= CURVE_HA_TH)
	{
		isCurve = true;
		cnt_curve = 0;
	}
	if (cnt_curve >= TIMEOUTVAL_CURVE)
	{
		isCurve = false;
	}

	// 出力
	handle(handleVal);
	motor(leftMotorVal, rightMotorVal);
}

/************************************************************************/
/**
 * クロス/ハーフラインの検出.
 */
void getLine_pe(void)
{
	int lineLeftMax = 0;
	int lineRightMax = 0;
	int crossLineMax = 0;
	int leftRate = 0;
	int rightRate = 0;

	// harfLineNmberから飛ばしで何行か調べる(10～90)
	for (uint8_t y = 20; y <  imgSizeY - 5 ; y += 1)
	{
		checkLine(y, leftRate, rightRate);
		if (lineLeftMax <= leftRate)
		{
			lineLeftMax = leftRate;
		}
		if (lineRightMax <= rightRate)
		{
			lineRightMax = rightRate;
		}
	}
	crossLineMax = checkCrossLine(lineLeftMax, lineRightMax);

	// 左ハーフライン
	if (lineLeftMax >= HALF_TH && cnt_Not_Rain_Change >= 1000)
	{
		isHalfLine_L = true;
	}
	else
	{
		isHalfLine_L = false;
	}
	// 右ハーフライン
	if (lineRightMax >= HALF_TH && cnt_Not_Rain_Change >= 1000)
	{
		isHalfLine_R = true;
	}
	else
	{
		isHalfLine_R = false;
	}
	// クロスライン
	if (crossLineMax >= HALF_TH && cnt_Not_Rain_Change >= 1000)
	{
		isCrossLine = true;
	}
	else
	{
		isCrossLine = false;
	}
}

/************************************************************************/
/**
 * クロス/ハーフラインの検出(前センサ).
 */
void getFrontLine_pe(void)
{
	int lineLeftMax = 0;
	int lineRightMax = 0;
	int crossLineMax = 0;
	int lineCheckCnt = 0;
	int lineCnt = 0;

	for (int y = 30; y <  imgSizeY * 0.89 ; y += 1)
	{
		int leftRate, rightRate;
		checkFrontLine(y, leftRate, rightRate);
		if (lineLeftMax <= leftRate)
		{
			lineLeftMax = leftRate;
		}
		if (lineRightMax <= rightRate)
		{
			lineRightMax = rightRate;
		}
		lineCheckCnt += 1;
		if (lineData[y].isLineTrue == true)
		{
			lineCnt += 1;
		}
	}
	crossLineMax = checkFrontCrossLine(lineLeftMax, lineRightMax);

	// 左ハーフライン
	if (lineLeftMax >= HALF_TH)
	{
		isfrontHalfLine_L = true;
	}
	else
	{
		isfrontHalfLine_L = false;
	}

	// 右ハーフライン
	if (lineRightMax >= HALF_TH)
	{
		isfrontHalfLine_R = true;
	}
	else
	{
		isfrontHalfLine_R = false;
	}

	// クロスライン
	if (crossLineMax >= HALF_TH)
	{
		isfrontCrossLine = true;
	}
	else
	{
		isfrontCrossLine = false;
	}

	// センターライン無し
	if (crossLineMax >= BLACK_TH)
	{
		isfrontLine = true;
	}
	else
	{
		isfrontLine = false;
	}

	frontLinePe = (lineCnt / lineCheckCnt) * 100;
}

/************************************************************************/
/**
 * 簡易的なクロス/ハーフラインの検出.
 * 
 *	@param[in]		y		ライン位置
 *
 *	@return			クロス/ハーフライン検出した場合 true
 */
bool simpleLineCheck(int y)
{
	// 一行の何パーセントが白かを計算
	int maxX = imgSizeX - 2;
	int ratio = (lineData[y].WHTcnt * 100) / maxX;
	if (WHT_TH <= ratio && !isCurve)
	{
		return true;
	}
	else
	{
		return false;
	}
}

/************************************************************************/
/**
 * ハーフラインの範囲取得.
 */
void makeRange(void)
{
	for (uint8_t y = 0; y < imgSizeY; y++)
	{
		HL_Data[y].yRange  = y * YRGAIN;
		HL_Data[y].xRange = (y * XRGAIN) + 40;
	}
}

/************************************************************************/
/**
 * Y座標範囲取得.
 * 
 *	@param[in]		yData		ライン位置
 *
 *	@return			Y座標の範囲
 */
int rangCcheck_y(int yData)
{
	int ret_range = 0;
	if ((int16_t)(HL_Data[yData].yRange + yData) >= (int8_t)(imgSizeY - 1))
	{
		ret_range = (imgSizeY - 1) - yData;
	}
	else
	{
		ret_range = HL_Data[yData].yRange;
	}
	return ret_range;
}

/************************************************************************/
/**
 * 復帰ライン確認.
 *
 *	@return		復帰条件を満たしている：true
 */
int checkReturnLine(void)
{
	if (isUpdateLine)
	{
		return true;
	}
	else
	{
		switch (sensor_inp(&sensor0, MASK4_4))
		{
		// ○○●● ●●○○
		case 0x3c:
		// ○○○● ●●○○
		case 0x1c:
		// ○○●● ●○○○
		case 0x38:
		// ○○○● ●○○○
		case 0x18:
		// ○○●○ ○○○○
		case 0x20:
		// ○○○○ ○●○○
		case 0x04:
		// ○○○● ○○○○
		case 0x10:
		// ○○○○ ●○○○
		case 0x08:
			return true;
			break;
		default:
			return false;
			break;
		}
	}
}

/************************************************************************/
/**
 * ライン検出処理.
 * 
 *	@param[in]		yData		ライン位置
 *	@param[out]		leftRate	左側のライン一致率
 *	@param[out]		rightRate	右側のライン一致率
 */
void checkLine(int yData, int &leftRate, int &rightRate)
{
	// カメラの左端
	int maxLeft = 1;
	// カメラの右端
	int maxRight = imgSizeX - 1;
	// 範囲
	int yrange = rangCcheck_y(yData);
	// 確認用
	HALF_LINE_CHECK halfLine_L[imgSizeY] = {};
	HALF_LINE_CHECK halfLine_R[imgSizeY] = {};

	uint16_t trueCnt, searchCnt;

	for (int y = yData; y < yData + yrange; y += 1)
	{
		uint8_t centerPoint_x = imgSizeX << 1;
		if (lineData[y].centerCnt > 0)
		{
			if (centerPoint[y] < imgSizeX)
			{
				centerPoint_x = centerPoint[y];
			}
		}

		trueCnt = 0;
		searchCnt = 0;

		// 左側
		for (uint8_t x = maxLeft; x < centerPoint_x ; x += 1)
		{
			// 白だった回数
			if (diffData[y][x].camera >= HL_Data[y].Th)
			{
				trueCnt++;
			}
			// 調べた回数
			searchCnt++;
		}
		halfLine_L[yData].trueCnt  = trueCnt;
		halfLine_L[yData].checkcnt = searchCnt;

		trueCnt = 0;
		searchCnt = 0;
		// 右側
		for (uint8_t x = centerPoint_x; x < maxRight; x += 1)
		{
			// 白だった回数
			if (diffData[y][x].camera >= HL_Data[y].Th)
			{
				trueCnt++;
			}
			// 調べた回数
			searchCnt++;
		}
		halfLine_R[yData].trueCnt  = trueCnt;
		halfLine_R[yData].checkcnt = searchCnt;
	}

	if (!isCurve && (halfLine_L[yData].checkcnt > 0) && (halfLine_R[yData].checkcnt > 0))
	{
		// パーセント計算
		if (halfLine_L[yData].checkcnt > 0)
		{
			leftRate  = (halfLine_L[yData].trueCnt * 100) / halfLine_L[yData].checkcnt;
		}
		else
		{
			leftRate = 0;
		}
		if (halfLine_R[yData].checkcnt > 0)
		{
			rightRate = (halfLine_R[yData].trueCnt * 100) / halfLine_R[yData].checkcnt;
		}
		else
		{
			rightRate = 0;
		}
	}
	else
	{
		leftRate = 0;
		rightRate = 0;
	}
}

/************************************************************************/
/**
 * クロスライン検出処理.
 * 
 *	@return			1：クロスラインあり / 0：クロスライン無し
 */
inline int checkCrossLine(int lineLeftMax, int lineRightMax)
{
	// 平均化
	int ratio = (lineLeftMax + lineRightMax) / 2;
	if (!isCurve)
	{
		return ratio;
	}
	else
	{
		return 0;
	}
}

/************************************************************************/
/**
 * ライン検出処理.
 * 
 *	@param[in]		yData		ライン位置
 *	@param[out]		leftRate	左側のライン一致率
 *	@param[out]		rightRate	右側のライン一致率
 */
void checkFrontLine(int yData, int &leftRate, int &rightRate)
{
	int centerPoint_x = (imgSizeX - 2) / 2;
	// 左端
	int maxLeft = int(imgSizeX * 0.25);
	// 右端
	int maxRight = int(imgSizeX * 0.75);
	// 範囲
	int yrange = rangCcheck_y(yData);
	// 確認用
	HALF_LINE_CHECK frontHalfLine_L[imgSizeY] = {};
	HALF_LINE_CHECK frontHalfLine_R[imgSizeY] = {};

	for (int y = yData; y < yData + yrange; y += 1)
	{
		int x;
		// 左
		for (x = maxLeft; x < centerPoint_x ; x += 1)
		{
			if (diffData[y][x].camera >= HL_Data[y].Th)
			{
				// 白だった回数
				frontHalfLine_L[yData].trueCnt += 1;
			}
			// 調べた回数
			frontHalfLine_L[yData].checkcnt += 1;
		}
		// 右
		for (x = centerPoint_x; x < maxRight ; x += 1)
		{
			if (diffData[y][x].camera >= HL_Data[y].Th)
			{
				// 白だった回数
				frontHalfLine_R[yData].trueCnt += 1;
			}
			// 調べた回数
			frontHalfLine_R[yData].checkcnt += 1;
		}
	}
	if (!isCurve)
	{
		leftRate  = (frontHalfLine_L[yData].trueCnt * 100) / frontHalfLine_L[yData].checkcnt;
		rightRate = (frontHalfLine_R[yData].trueCnt * 100) / frontHalfLine_R[yData].checkcnt;
	}
	else
	{
		leftRate  = 0;
		rightRate = 0;
	}
}

/************************************************************************/
/**
 * クロスライン検出処理.
 * 
 *	@return			1：クロスラインあり / 0：クロスライン無し
 */
inline int checkFrontCrossLine(int lineLeftMax, int lineRightMax)
{
	// 平均化
	int ratio = (lineLeftMax + lineRightMax) / 2;
	if (!isCurve)
	{
		return ratio;
	}
	else
	{
		return 0;
	}
}

/************************************************************************/
/**
 * 自動二値化処理.
 * 
 *	@param[in]		line		二値化行数(0 ～ 119)
 *	@param[in]		threshold	二値化閾値
 *	@param[in]		diff		自動閾値用差
 *
 *	@return			8点の二値化結果
 */
unsigned char convertBinarization(int line, int threshold, int diff)
{
	// 8点の値
	int d[8];
	int sa_7_6, sa_6_5, sa_5_4, sa_4_3, sa_3_2, sa_2_1, sa_1_0;
	unsigned char ret;

	// 8点の画素データ値を代入
	d[7] = getImage(  31, line);
	d[6] = getImage(  43, line);
	d[5] = getImage(  54, line);
	d[4] = getImage(  71, line);
	d[3] = getImage(  88, line);
	d[2] = getImage( 105, line);
	d[1] = getImage( 116, line);
	d[0] = getImage( 128, line);

	int min = d[0];
	int max = d[0];
	for (uint8_t i = 1; i < 8; i++ )
	{
		if (max <= d[i])
		{
			max = d[i]; // 8個のうち、最大を見つける
		}
		if (min >= d[i])
		{
			min = d[i]; // 8個のうち、最小を見つける
		}
	}

	// 隣同士の差の絶対値
	sa_7_6 = abs(d[7] - d[6]);
	sa_6_5 = abs(d[6] - d[5]);
	sa_5_4 = abs(d[5] - d[4]);
	sa_4_3 = abs(d[4] - d[3]);
	sa_3_2 = abs(d[3] - d[2]);
	sa_2_1 = abs(d[2] - d[1]);
	sa_1_0 = abs(d[1] - d[0]);

	// 最終的な閾値
	int calThreshold;
	if (max >= threshold)
	{
		// 最大値がs以上なら、sをしきい値とする
		calThreshold = threshold;
	}
	else if (sa_7_6 >= diff || sa_6_5 >= diff || sa_5_4 >= diff ||
			 sa_4_3 >= diff || sa_3_2 >= diff || sa_2_1 >= diff || sa_1_0 >= diff)
	{
		// 隣同士の差が１つでも「差」以上なら、８点の（最大値－最小値）×0.7 + 最小値　をしきい値とする
		calThreshold =  ( max - min ) * 7 / 10 + min;
	}
	else
	{
		// 当てはまらなければ、しきい値を256とする、すなわちすべて0となる
		calThreshold = 256;
	}

	// d[7]～d[0]をbit7～bit0に割り当てる
	ret = 0;
	for (int8_t i = 7; i >= 0; i--)
	{
		ret <<= 1;
		ret |= (d[i] >= calThreshold ? 1 : 0);
	}

	return ret;
}

/************************************************************************/
/**
 * charデータをlog変数に二進数で変換.
 * 
 *	@param[in]		hex		変換元データ
 *
 *	@return			変換後のデータ
 */
unsigned long convertCharToLong(unsigned char hex)
{
	unsigned long  l = 0;

	for (uint8_t i = 0; i < 8; i++)
	{
		l *= 10;
		if (hex & 0x80)
		{
			l += 1;
		}
		hex <<= 1;
	}

	return l;
}

/************************************************************************/
/**
 * ログをRAMに保存.
 * 
 *	@param[in]		ImgData		保存画素データ
 */
void LOG_rec(mcrMat *ImData)
{
	if (sdlog_enable == 0)
		return;

	if (log_no < LOG_NUM)
	{
		log_data[log_no].pattern		= pattern;
		log_data[log_no].cnt_stop		= cnt_stop;
		log_data[log_no].center_diff	= cameraCenter - centerDiffAve;
		log_data[log_no].handle			= angle_buff;
		log_data[log_no].sens			= sensor_inp(&sensor0, MASK4_4);
		log_data[log_no].speedL			= motor_buff_l;
		log_data[log_no].speedR			= motor_buff_r;
		log_data[log_no].flag1			= encoderInfo.speed;
		log_data[log_no].flag2			= encoderInfo.mTotalDist;

		uint8_t point = 0;
		// 中央線の保存        中央線を見つけるプログラムがあればいれれる
		for (int center_posi = 0; center_posi < 30; center_posi++, point += 2)
		{
			// @NOTE 横幅30ビットに納めないとログとして確認できない
			uint16_t center = (centerPoint[point] + centerPoint[point + 1]) / 4;
			if (center > 30)
			{
				log_data[log_no].center_position[center_posi] = 0;
			}
			else
			{
				log_data[log_no].center_position[center_posi] = center;
			}
		}

		int n = 0;
		uint16_t addCnt = (ImData->w / 20);
		unsigned char d1, d2;
		// 画像の保存
		for (int y = 9; y < 29; y++)
		{
			for (int x = 0; x < (ImData->w); x = x + addCnt)
			{
				d1 = (ImData->y[(y * ImData->w) + x] & 0xf0);
				d2 = (ImData->y[(y * ImData->w) + x + 1]) >> 4;
				log_data[log_no].gaso[n++] = d1 | d2;
			}
		}
	}

	if (log_no > LOG_NUM)
	{
		sdlog_enable = 0; //記録終了
	}
	log_no++;
}

void SD_LOG_Write(void)
{

	sdlog_enable = 0; //記録終了

	if (sd_enable == 0)
		return;

	pc.printf("file make\n");

	char fname[32];
	int no = 0;
	char DIRPATH[] = "/storage";
	DIR *dir;
	struct dirent *entry;

	dir = opendir(DIRPATH);
	pc.printf("open dir\n");
	if (dir != NULL)
	{
		while ((entry = readdir(dir)) != NULL)
		{
			no++;
		}
	}
	pc.printf("close dir\n");
	closedir(dir);

	sprintf(fname, "/storage/test%05d.csv", no);

	pc.printf("file write\n");

	// ファイルオープン
	fp = fopen(fname, "w");
	// SD書き込みテスト
	if (fp)
	{
		sd_enable = true;
	}
	else
	{
		sd_enable = false;
		pc.printf("SD FILE OPEN ERROR");
	}

	for (uint16_t i = 0; i < log_no; i++)
	{
		fprintf(fp, "%d,", log_data[i].pattern);
		fprintf(fp, "%d,", log_data[i].cnt_stop);
		fprintf(fp, "%d,", log_data[i].center_diff);
		fprintf(fp, "%d,", log_data[i].sens);
		fprintf(fp, "%d,", log_data[i].handle);
		fprintf(fp, "%d,", log_data[i].speedL);
		fprintf(fp, "%d,", log_data[i].speedR);
		fprintf(fp, "%d,", log_data[i].flag1);
		fprintf(fp, "%d,", log_data[i].flag2);
		for (int center_posi = 10; center_posi < 27; center_posi++)
		{
			fprintf(fp, "%d,", log_data[i].center_position[center_posi]);
		}
		for (int j = 0; j < LOG_GASO_BYTES; j++)
		{
			fprintf(fp, "%d,", log_data[i].gaso[j]);
		}
		fprintf(fp, "\n");
	}
	fclose(fp);
}

//------------------------------------------------------------------//
// End of file
//------------------------------------------------------------------//
