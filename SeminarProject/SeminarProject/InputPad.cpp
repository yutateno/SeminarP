#include "InputPad.hpp"

using namespace MYINPUTPAD;

// static変数の宣言
unsigned __int8 InputPad::controllerNum;
int InputPad::button[4][16];
int InputPad::stick[4][4];
XINPUT_STATE InputPad::state[4];
XINPUT_STICK_MY_DEADZONE InputPad::stickDeadZone;
bool InputPad::setControll[4];
XINPUT_VIBRATION InputPad::vibration;


// コンストラクタ
InputPad::InputPad()
{
	// コントローラ分ループ
	InputPad::controllerNum = 0;
	for (int i = 0; i != 4; ++i)
	{
		if (XInputGetState(i, &InputPad::state[i]) == ERROR_SUCCESS)
		{
			InputPad::controllerNum++;
			InputPad::setControll[i] = true;
		}
		else
		{
			InputPad::setControll[i] = false;
		}
	}
	for (int i = 0; i != InputPad::controllerNum; ++i)
	{
		ZeroMemory(&InputPad::state[i], sizeof(XINPUT_STATE));
		// ボタンを0に初期化(スティックはデッドゾーンの関係で行わない)
		for (int j = 0; j != 16; ++j)
		{
			InputPad::button[i][j] = 0;
		}
	}
}

// デストラクタ
InputPad::~InputPad()
{
	for (int i = 0; i != InputPad::controllerNum; ++i)
	{
		ZeroMemory(&InputPad::state[i], sizeof(XINPUT_STATE));
	}
}


// 更新
void InputPad::Update()
{
	// コントローラの数だけ行う
	for (int i = 0; i != InputPad::controllerNum; ++i)
	{
		if (InputPad::setControll[i])		// 接続されているコントローラーのみ判断するように
		{
			ZeroMemory(&InputPad::state[i], sizeof(XINPUT_STATE));
			XInputGetState(i, &InputPad::state[i]);
			// ボタン操作
			for (int j = 0; j != 16; ++j)
			{
				if (j == 10 || j == 11)		// xinput.h上で割り当てられていない
				{
					continue;
				}
				if ((InputPad::state[i].Gamepad.wButtons & (int)pow(2.0, (double)j)) != 0)		// ボタンを押したら
				{
					InputPad::button[i][j]++;
					if (InputPad::button[i][j] >= 1000)		// いつか上限値行くと思うので回避
					{
						InputPad::button[i][j] = 2;
					}
				}
				else if (InputPad::button[i][j] > 0)	// 押していたボタンを離したら
				{
					InputPad::button[i][j] = -1;
				}
				else		// ボタンに触れていない
				{
					InputPad::button[i][j] = 0;
				}
			}

			// スティック操作
			// 左スティックの左右(公式のデッドゾーンを使わせてもらうが自作する方が操作性よろしくなると思われる)
			if (InputPad::state[i].Gamepad.sThumbLX > stickDeadZone.LEFT_AXIS_X_RIGHT
				|| InputPad::state[i].Gamepad.sThumbLX < stickDeadZone.LEFT_AXIS_X_LEFT)		// スティックを操作したら
			{
				InputPad::stick[i][XINPUT_PAD::STICK_LEFT_AXIS_X] = InputPad::state[i].Gamepad.sThumbLX;
			}
			else		// スティックを操作していない
			{
				InputPad::stick[i][XINPUT_PAD::STICK_LEFT_AXIS_X] = 0;
			}
			// 左スティックの上下
			if (InputPad::state[i].Gamepad.sThumbLY > stickDeadZone.LEFT_AXIS_Y_UP
				|| InputPad::state[i].Gamepad.sThumbLY < stickDeadZone.LEFT_AXIS_Y_DOWN)		// スティックを操作したら
			{
				InputPad::stick[i][XINPUT_PAD::STICK_LEFT_AXIS_Y] = InputPad::state[i].Gamepad.sThumbLY;
			}
			else		// スティックを操作していない
			{
				InputPad::stick[i][XINPUT_PAD::STICK_LEFT_AXIS_Y] = 0;
			}
			// 右スティックの左右
			if (InputPad::state[i].Gamepad.sThumbRX > stickDeadZone.RIGHT_AXIS_X_RIGHT
				|| InputPad::state[i].Gamepad.sThumbRX < stickDeadZone.RIGHT_AXIS_X_LEFT)		// スティックを操作したら
			{
				InputPad::stick[i][XINPUT_PAD::STICK_RIGHT_AXIS_X] = InputPad::state[i].Gamepad.sThumbRX;
			}
			else		// スティックを操作していない
			{
				InputPad::stick[i][XINPUT_PAD::STICK_RIGHT_AXIS_X] = 0;
			}
			// 右スティックの上下
			if (InputPad::state[i].Gamepad.sThumbRY > stickDeadZone.RIGHT_AXIS_Y_UP
				|| InputPad::state[i].Gamepad.sThumbRY < stickDeadZone.RIGHT_AXIS_Y_DOWN)		// スティックを操作したら
			{
				InputPad::stick[i][XINPUT_PAD::STICK_RIGHT_AXIS_Y] = InputPad::state[i].Gamepad.sThumbRY;
			}
			else		// スティックを操作していない
			{
				InputPad::stick[i][XINPUT_PAD::STICK_RIGHT_AXIS_Y] = 0;
			}
		}
	}
}


// バイブレーションを行う
void InputPad::Vibration(unsigned __int8 use_padnum, int time, unsigned __int16 rightVib, unsigned __int16 leftVib)
{
	// バイブレーション値
	InputPad::vibration.wRightMotorSpeed = rightVib;
	InputPad::vibration.wLeftMotorSpeed = leftVib;

	XInputSetState(use_padnum, &InputPad::vibration);		// バーブレーション値を設定
}


// バイブレーションを止める
void InputPad::VibrationStop(unsigned __int8 use_padnum)
{
	// バイブレーション値
	InputPad::vibration.wRightMotorSpeed = 0;				// 0にする
	InputPad::vibration.wLeftMotorSpeed = 0;				// 0にする

	XInputSetState(use_padnum, &InputPad::vibration);		// バイブレーション値を設定
}


// コントローラーの繋がっている数
int InputPad::GetPadNum()
{
	return (int)InputPad::controllerNum;
}

// ボタン入力取得
int InputPad::GetPadButtonData(unsigned __int8 use_padnum, unsigned __int8 use_button)
{
	return InputPad::button[use_padnum][use_button];
}

// トリガー入力取得
int InputPad::GetPadTriggerData(unsigned __int8 use_padnum, bool use_Is_triggerLeft)
{
	if (use_Is_triggerLeft)
	{
		return InputPad::state[use_padnum].Gamepad.bLeftTrigger;
	}
	else
	{
		return InputPad::state[use_padnum].Gamepad.bRightTrigger;
	}
}

// スティック入力取得
int InputPad::GetPadThumbData(unsigned __int8 use_padnum, unsigned __int8 use_stick)
{
	return InputPad::stick[use_padnum][use_stick];
}

// スティックのデッドゾーン設定
void InputPad::SetPadDeadZone(short leftPad_right, short leftPad_left
	, short leftPad_up, short leftPad_down, short rightPad_right, short rightPad_left, short rightPad_up, short rightPad_down)
{
	InputPad::stickDeadZone.LEFT_AXIS_X_RIGHT = (leftPad_right == XINPUT_GAMEPAD_LEFT_THUMB_DEADZONE) ? XINPUT_GAMEPAD_LEFT_THUMB_DEADZONE : leftPad_right;
	InputPad::stickDeadZone.LEFT_AXIS_X_LEFT = (leftPad_left == -XINPUT_GAMEPAD_LEFT_THUMB_DEADZONE) ? -XINPUT_GAMEPAD_LEFT_THUMB_DEADZONE : leftPad_left;
	InputPad::stickDeadZone.LEFT_AXIS_Y_UP = (leftPad_up == XINPUT_GAMEPAD_LEFT_THUMB_DEADZONE) ? XINPUT_GAMEPAD_LEFT_THUMB_DEADZONE : leftPad_up;
	InputPad::stickDeadZone.LEFT_AXIS_Y_DOWN = (leftPad_down == -XINPUT_GAMEPAD_LEFT_THUMB_DEADZONE) ? -XINPUT_GAMEPAD_LEFT_THUMB_DEADZONE : leftPad_down;
	InputPad::stickDeadZone.RIGHT_AXIS_X_RIGHT = (rightPad_right == XINPUT_GAMEPAD_RIGHT_THUMB_DEADZONE) ? XINPUT_GAMEPAD_RIGHT_THUMB_DEADZONE : rightPad_right;
	InputPad::stickDeadZone.RIGHT_AXIS_X_LEFT = (rightPad_left == -XINPUT_GAMEPAD_RIGHT_THUMB_DEADZONE) ? -XINPUT_GAMEPAD_RIGHT_THUMB_DEADZONE : rightPad_left;
	InputPad::stickDeadZone.RIGHT_AXIS_Y_UP = (rightPad_up == XINPUT_GAMEPAD_RIGHT_THUMB_DEADZONE) ? XINPUT_GAMEPAD_RIGHT_THUMB_DEADZONE : rightPad_up;
	InputPad::stickDeadZone.RIGHT_AXIS_Y_DOWN = (rightPad_down == -XINPUT_GAMEPAD_RIGHT_THUMB_DEADZONE) ? -XINPUT_GAMEPAD_RIGHT_THUMB_DEADZONE : rightPad_down;
}
