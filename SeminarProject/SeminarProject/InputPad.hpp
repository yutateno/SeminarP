#pragma once
#ifndef _MY_INPUTPAD_H
#define _MY_INPUTPAD_H


#include <Windows.h>
#include <math.h>


namespace MY_XINPUT	// XINPUT_STATEがあいまいとなるので一括スコープ逃げ
{

#ifndef _COMPILE_SLOPE					// 気持ち悪いから定義除けする
#define _COMPILE_SLOPE
#include <Xinput.h>

#pragma comment(lib, "xinput.lib")

#endif // !_COMPILE_SLOPE


	namespace 
	{
		// 番号
		const unsigned __int8 NUM01 = 0;
		const unsigned __int8 NUM02 = 1;
		const unsigned __int8 NUM03 = 2;
		const unsigned __int8 NUM04 = 3;


		// ボタン
		const unsigned __int8 BUTTON_UP			 = 0;
		const unsigned __int8 BUTTON_DOWN		 = 1;
		const unsigned __int8 BUTTON_LEFT		 = 2;
		const unsigned __int8 BUTTON_RIGHT		 = 3;

		const unsigned __int8 BUTTON_START		 = 4;
		const unsigned __int8 BUTTON_BACK		 = 5;

		const unsigned __int8 BUTTON_STICK_LEFT	 = 6;
		const unsigned __int8 BUTTON_STICK_RIGHT = 7;

		const unsigned __int8 BUTTON_A			 = 12;
		const unsigned __int8 BUTTON_B			 = 13;
		const unsigned __int8 BUTTON_X			 = 14;
		const unsigned __int8 BUTTON_Y			 = 15;


		// トリガ
		const bool TRIGGER_RT = 0;
		const bool TRIGGER_LT = 1;

		const unsigned __int8 SHOULDER_LB = 8;
		const unsigned __int8 SHOULDER_RB = 9;


		// スティック
		const unsigned __int8 STICK_RIGHT_X	 = 0;
		const unsigned __int8 STICK_RIGHT_Y	 = 1;
		const unsigned __int8 STICK_LEFT_X	 = 2;
		const unsigned __int8 STICK_LEFT_Y	 = 3;

		const int MAX_STICK_PLUS = 32767;
		const int MAX_STICK_MINUS = -32768;


		// バイブレーション
		const unsigned __int16 VIB_MAX = 65535;
	}

	struct STICK_DEADZONE
	{
		// 左スティックの左右
		short LEFT_RIGHT = XINPUT_GAMEPAD_LEFT_THUMB_DEADZONE;
		short LEFT_LEFT	 = -XINPUT_GAMEPAD_LEFT_THUMB_DEADZONE;


		// 左スティックの上下
		short LEFT_UP	 = XINPUT_GAMEPAD_LEFT_THUMB_DEADZONE;
		short LEFT_DOWN	 = -XINPUT_GAMEPAD_LEFT_THUMB_DEADZONE;


		// 右スティックの左右
		short RIGHT_RIGHT	 = XINPUT_GAMEPAD_RIGHT_THUMB_DEADZONE;
		short RIGHT_LEFT	 = -XINPUT_GAMEPAD_RIGHT_THUMB_DEADZONE;


		// 右スティックの上下
		short RIGHT_UP	 = XINPUT_GAMEPAD_RIGHT_THUMB_DEADZONE;
		short RIGHT_DOWN = -XINPUT_GAMEPAD_RIGHT_THUMB_DEADZONE;
	};


	class InputPad
	{
	private:
		static unsigned __int8 controllerNum;		// 接続している最大の個数
		static bool setControll[4];					// 使える操作の番号


		static unsigned __int8 playerPadNum;		// プレイヤーが使うコントローラー


		static int button[4][16];					// wButtonの対応
		static int stick[4][4];						// stickの対応(公式だとthumb)


		static XINPUT_STATE state[4];				// xinputの中身


		static STICK_DEADZONE stickDeadZone;		// スティックのデッドゾーン値


		static XINPUT_VIBRATION vibration;			// バイブレーションの構造体
		

	public:
		InputPad();			// コンストラクタ
		~InputPad();		// デストラクタ


		static void FirstUpdate();			// ゲーム開始前操作更新
		static void EverUpdate();			// ゲーム開始後操作更新


		static void Vibration(const unsigned __int8 use_padnum, const int time = 0
			, const unsigned __int16 rightVib = VIB_MAX, const unsigned __int16 leftVib = VIB_MAX);		// バイブレーションを行う

		static void VibrationStop(const unsigned __int8 use_padnum);								// バーブレーションを止める


		static void SetPlayerPadNum(const unsigned __int8 playerPadNum);					// プレイヤーの番号を設定


		static void SetPadDeadZone(const short leftPad_right = XINPUT_GAMEPAD_LEFT_THUMB_DEADZONE, const short leftPad_left = -XINPUT_GAMEPAD_LEFT_THUMB_DEADZONE
			, const short leftPad_up	 = XINPUT_GAMEPAD_LEFT_THUMB_DEADZONE,	 const short leftPad_down = -XINPUT_GAMEPAD_LEFT_THUMB_DEADZONE
			, const short rightPad_right = XINPUT_GAMEPAD_RIGHT_THUMB_DEADZONE,	 const short rightPad_left = -XINPUT_GAMEPAD_RIGHT_THUMB_DEADZONE
			, const short rightPad_up	 = XINPUT_GAMEPAD_RIGHT_THUMB_DEADZONE,	 const short rightPad_down = -XINPUT_GAMEPAD_RIGHT_THUMB_DEADZONE);		// デッドゾーンの設定値変更  // (ある程度楽したいので一応デフォルト引数乱用)

		static void InitPadDeadZone();			// デッドゾーンを初期値に戻す


		static const int GetPadNum();																			// コントローラの数
		static const int GetPadButtonData(const unsigned __int8 use_padnum, const unsigned __int8 use_button);			// コントローラのボタン操作
		static const int GetPadTriggerData(const unsigned __int8 use_padnum, const bool use_Is_triggerLeft);				// コントローラのトリガー操作
		static const int GetPadThumbData(const unsigned __int8 use_padnum, const unsigned __int8 use_stick);				// コントローラのスティック操作
		static const short GetPadThumbMax(const bool stickLightNow, const bool stickXAxisNow, const bool stickPlusNow);
	};
}

#endif // !_MY_INPUTPAD_H