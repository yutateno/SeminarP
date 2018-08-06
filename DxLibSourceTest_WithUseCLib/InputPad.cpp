#include "InputPad.hpp"

using namespace MYINPUTPAD;

// static�ϐ��̐錾
unsigned __int8 InputPad::controllerNum;
int InputPad::button[4][16];
int InputPad::stick[4][4];
XINPUT_STATE InputPad::state[4];
XINPUT_STICK_MY_DEADZONE InputPad::stickDeadZone;

InputPad::InputPad()
{
	// �R���g���[�������[�v
	InputPad::controllerNum = 0;
	for (int i = 0; i < 4; ++i)
	{
		if (XInputGetState(i, &InputPad::state[i]) == ERROR_SUCCESS)
		{
			InputPad::controllerNum++;
		}
	}
	for (int i = 0; i != InputPad::controllerNum; ++i)
	{
		ZeroMemory(&InputPad::state[i], sizeof(XINPUT_STATE));
		// �{�^����0�ɏ�����(�X�e�B�b�N�̓f�b�h�]�[���̊֌W�ōs��Ȃ�)
		for (int j = 0; j != 16; ++j)
		{
			InputPad::button[i][j] = 0;
		}
	}
}

InputPad::~InputPad()
{
	for (int i = 0; i != InputPad::controllerNum; ++i)
	{
		ZeroMemory(&InputPad::state[i], sizeof(XINPUT_STATE));
	}
}

void InputPad::Update()
{
	// �R���g���[���̐������s��
	for (int i = 0; i < controllerNum; i++)
	{
		ZeroMemory(&InputPad::state[i], sizeof(XINPUT_STATE));
		if (XInputGetState(i, &InputPad::state[i]) == ERROR_SUCCESS)		// �ڑ�����Ă���
		{
			// �{�^������
			for (int j = 0; j != 16; ++j)
			{
				if (j == 10 || j == 11)		// xinput.h��Ŋ��蓖�Ă��Ă��Ȃ�
				{
					continue;
				}
				if ((InputPad::state[i].Gamepad.wButtons & (int)pow(2.0, (double)j)) != 0)		// �{�^������������
				{
					InputPad::button[i][j]++;
					if (InputPad::button[i][j] >= 1000)		// ��������l�s���Ǝv���̂ŉ��
					{
						InputPad::button[i][j] = 2;
					}
				}
				else if (InputPad::button[i][j] > 0)	// �����Ă����{�^���𗣂�����
				{
					InputPad::button[i][j] = -1;
				}
				else		// �{�^���ɐG��Ă��Ȃ�
				{
					InputPad::button[i][j] = 0;
				}
			}

			// �X�e�B�b�N����
			// ���X�e�B�b�N�̍��E(�����̃f�b�h�]�[�����g�킹�Ă��炤�����삷��������쐫��낵���Ȃ�Ǝv����)
			if (InputPad::state[i].Gamepad.sThumbLX > stickDeadZone.LEFT_AXIS_X_RIGHT
				|| InputPad::state[i].Gamepad.sThumbLX < stickDeadZone.LEFT_AXIS_X_LEFT)		// �X�e�B�b�N�𑀍삵����
			{
				InputPad::stick[i][XINPUT_PAD::STICK_LEFT_AXIS_X] = InputPad::state[i].Gamepad.sThumbLX;
			}
			else		// �X�e�B�b�N�𑀍삵�Ă��Ȃ�
			{
				InputPad::stick[i][XINPUT_PAD::STICK_LEFT_AXIS_X] = 0;
			}
			// ���X�e�B�b�N�̏㉺
			if (InputPad::state[i].Gamepad.sThumbLY > stickDeadZone.LEFT_AXIS_Y_UP
				|| InputPad::state[i].Gamepad.sThumbLY < stickDeadZone.LEFT_AXIS_Y_DOWN)		// �X�e�B�b�N�𑀍삵����
			{
				InputPad::stick[i][XINPUT_PAD::STICK_LEFT_AXIS_Y] = InputPad::state[i].Gamepad.sThumbLY;
			}
			else		// �X�e�B�b�N�𑀍삵�Ă��Ȃ�
			{
				InputPad::stick[i][XINPUT_PAD::STICK_LEFT_AXIS_Y] = 0;
			}
			// �E�X�e�B�b�N�̍��E
			if (InputPad::state[i].Gamepad.sThumbRX > stickDeadZone.RIGHT_AXIS_X_RIGHT
				|| InputPad::state[i].Gamepad.sThumbRX < stickDeadZone.RIGHT_AXIS_X_LEFT)		// �X�e�B�b�N�𑀍삵����
			{
				InputPad::stick[i][XINPUT_PAD::STICK_RIGHT_AXIS_X] = InputPad::state[i].Gamepad.sThumbRX;
			}
			else		// �X�e�B�b�N�𑀍삵�Ă��Ȃ�
			{
				InputPad::stick[i][XINPUT_PAD::STICK_RIGHT_AXIS_X] = 0;
			}
			// �E�X�e�B�b�N�̏㉺
			if (InputPad::state[i].Gamepad.sThumbRY > stickDeadZone.RIGHT_AXIS_Y_UP
				|| InputPad::state[i].Gamepad.sThumbRY < stickDeadZone.RIGHT_AXIS_Y_DOWN)		// �X�e�B�b�N�𑀍삵����
			{
				InputPad::stick[i][XINPUT_PAD::STICK_RIGHT_AXIS_Y] = InputPad::state[i].Gamepad.sThumbRY;
			}
			else		// �X�e�B�b�N�𑀍삵�Ă��Ȃ�
			{
				InputPad::stick[i][XINPUT_PAD::STICK_RIGHT_AXIS_Y] = 0;
			}
		}
		else	// �ڑ�����Ă��Ȃ�
		{
			ZeroMemory(&InputPad::state[i], sizeof(XINPUT_STATE));
		}
	}
}

int InputPad::GetPadNum()
{
	return (int)InputPad::controllerNum;
}

int InputPad::GetPadButtonData(unsigned __int8 use_padnum, unsigned __int8 use_button)
{
	return InputPad::button[use_padnum][use_button];
}

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

int InputPad::GetPadThumbData(unsigned __int8 use_padnum, unsigned __int8 use_stick)
{
	return InputPad::stick[use_padnum][use_stick];
}

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
