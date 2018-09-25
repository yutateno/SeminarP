#include "InputPad.hpp"

using namespace MY_XINPUT;

// static�ϐ��̐錾
unsigned __int8 InputPad::controllerNum;
unsigned __int8 InputPad::playerPadNum;
int InputPad::button[4][16];
int InputPad::stick[4][4];
::XINPUT_STATE InputPad::state[4];
STICK_DEADZONE InputPad::stickDeadZone;
bool InputPad::setControll[4];
XINPUT_VIBRATION InputPad::vibration;


// �R���X�g���N�^
InputPad::InputPad()
{
	// �R���g���[�������[�v
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
		ZeroMemory(&InputPad::state[i], sizeof(::XINPUT_STATE));
		// �{�^����0�ɏ�����(�X�e�B�b�N�̓f�b�h�]�[���̊֌W�ōs��Ȃ�)
		for (int j = 0; j != 16; ++j)
		{
			InputPad::button[i][j] = 0;
		}
	}
}

// �f�X�g���N�^
InputPad::~InputPad()
{
	for (int i = 0; i != InputPad::controllerNum; ++i)
	{
		ZeroMemory(&InputPad::state[i], sizeof(::XINPUT_STATE));
	}
}


// �X�V
void InputPad::FirstUpdate()
{
	// �R���g���[���̐������s��
	for (int i = 0; i != InputPad::controllerNum; ++i)
	{
		if (InputPad::setControll[i])		// �ڑ�����Ă���R���g���[���[�̂ݔ��f����悤��
		{
			ZeroMemory(&InputPad::state[i], sizeof(::XINPUT_STATE));
			XInputGetState(i, &InputPad::state[i]);
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
			if (InputPad::state[i].Gamepad.sThumbLX > stickDeadZone.LEFT_RIGHT
				|| InputPad::state[i].Gamepad.sThumbLX < stickDeadZone.LEFT_LEFT)		// �X�e�B�b�N�𑀍삵����
			{
				InputPad::stick[i][STICK_LEFT_X] = InputPad::state[i].Gamepad.sThumbLX;
			}
			else		// �X�e�B�b�N�𑀍삵�Ă��Ȃ�
			{
				InputPad::stick[i][STICK_LEFT_X] = 0;
			}
			// ���X�e�B�b�N�̏㉺
			if (InputPad::state[i].Gamepad.sThumbLY > stickDeadZone.LEFT_UP
				|| InputPad::state[i].Gamepad.sThumbLY < stickDeadZone.LEFT_DOWN)		// �X�e�B�b�N�𑀍삵����
			{
				InputPad::stick[i][STICK_LEFT_Y] = InputPad::state[i].Gamepad.sThumbLY;
			}
			else		// �X�e�B�b�N�𑀍삵�Ă��Ȃ�
			{
				InputPad::stick[i][STICK_LEFT_Y] = 0;
			}
			// �E�X�e�B�b�N�̍��E
			if (InputPad::state[i].Gamepad.sThumbRX > stickDeadZone.RIGHT_RIGHT
				|| InputPad::state[i].Gamepad.sThumbRX < stickDeadZone.RIGHT_LEFT)		// �X�e�B�b�N�𑀍삵����
			{
				InputPad::stick[i][STICK_RIGHT_X] = InputPad::state[i].Gamepad.sThumbRX;
			}
			else		// �X�e�B�b�N�𑀍삵�Ă��Ȃ�
			{
				InputPad::stick[i][STICK_RIGHT_X] = 0;
			}
			// �E�X�e�B�b�N�̏㉺
			if (InputPad::state[i].Gamepad.sThumbRY > stickDeadZone.RIGHT_UP
				|| InputPad::state[i].Gamepad.sThumbRY < stickDeadZone.RIGHT_DOWN)		// �X�e�B�b�N�𑀍삵����
			{
				InputPad::stick[i][STICK_RIGHT_Y] = InputPad::state[i].Gamepad.sThumbRY;
			}
			else		// �X�e�B�b�N�𑀍삵�Ă��Ȃ�
			{
				InputPad::stick[i][STICK_RIGHT_Y] = 0;
			}
		}
	}
}


// �X�V
void InputPad::EverUpdate()
{
	ZeroMemory(&InputPad::state[InputPad::playerPadNum], sizeof(::XINPUT_STATE));
	XInputGetState(InputPad::playerPadNum, &InputPad::state[InputPad::playerPadNum]);
	// �{�^������
	for (int j = 0; j != 16; ++j)
	{
		if (j == 10 || j == 11)		// xinput.h��Ŋ��蓖�Ă��Ă��Ȃ�
		{
			continue;
		}
		if ((InputPad::state[InputPad::playerPadNum].Gamepad.wButtons & (int)pow(2.0, (double)j)) != 0)		// �{�^������������
		{
			InputPad::button[InputPad::playerPadNum][j]++;
			if (InputPad::button[InputPad::playerPadNum][j] >= 1000)		// ��������l�s���Ǝv���̂ŉ��
			{
				InputPad::button[InputPad::playerPadNum][j] = 2;
			}
		}
		else if (InputPad::button[InputPad::playerPadNum][j] > 0)	// �����Ă����{�^���𗣂�����
		{
			InputPad::button[InputPad::playerPadNum][j] = -1;
		}
		else		// �{�^���ɐG��Ă��Ȃ�
		{
			InputPad::button[InputPad::playerPadNum][j] = 0;
		}
	}

	// �X�e�B�b�N����
	// ���X�e�B�b�N�̍��E(�����̃f�b�h�]�[�����g�킹�Ă��炤�����삷��������쐫��낵���Ȃ�Ǝv����)
	if (InputPad::state[InputPad::playerPadNum].Gamepad.sThumbLX > stickDeadZone.LEFT_RIGHT
		|| InputPad::state[InputPad::playerPadNum].Gamepad.sThumbLX < stickDeadZone.LEFT_LEFT)		// �X�e�B�b�N�𑀍삵����
	{
		if (InputPad::state[InputPad::playerPadNum].Gamepad.sThumbLX > 0)
		{
			InputPad::stick[InputPad::playerPadNum][STICK_LEFT_X] = InputPad::state[InputPad::playerPadNum].Gamepad.sThumbLX - InputPad::stickDeadZone.LEFT_RIGHT;
		}
		else
		{
			InputPad::stick[InputPad::playerPadNum][STICK_LEFT_X] = InputPad::state[InputPad::playerPadNum].Gamepad.sThumbLX - InputPad::stickDeadZone.LEFT_LEFT;
		}
	}
	else		// �X�e�B�b�N�𑀍삵�Ă��Ȃ�
	{
		InputPad::stick[InputPad::playerPadNum][STICK_LEFT_X] = 0;
	}
	// ���X�e�B�b�N�̏㉺
	if (InputPad::state[InputPad::playerPadNum].Gamepad.sThumbLY > stickDeadZone.LEFT_UP
		|| InputPad::state[InputPad::playerPadNum].Gamepad.sThumbLY < stickDeadZone.LEFT_DOWN)		// �X�e�B�b�N�𑀍삵����
	{
		if (InputPad::state[InputPad::playerPadNum].Gamepad.sThumbLY > 0)
		{
			InputPad::stick[InputPad::playerPadNum][STICK_LEFT_Y] = InputPad::state[InputPad::playerPadNum].Gamepad.sThumbLY - InputPad::stickDeadZone.LEFT_UP;
		}
		else
		{
			InputPad::stick[InputPad::playerPadNum][STICK_LEFT_Y] = InputPad::state[InputPad::playerPadNum].Gamepad.sThumbLY - InputPad::stickDeadZone.LEFT_DOWN;
		}
	}
	else		// �X�e�B�b�N�𑀍삵�Ă��Ȃ�
	{
		InputPad::stick[InputPad::playerPadNum][STICK_LEFT_Y] = 0;
	}
	// �E�X�e�B�b�N�̍��E
	if (InputPad::state[InputPad::playerPadNum].Gamepad.sThumbRX > stickDeadZone.RIGHT_RIGHT
		|| InputPad::state[InputPad::playerPadNum].Gamepad.sThumbRX < stickDeadZone.RIGHT_LEFT)		// �X�e�B�b�N�𑀍삵����
	{
		if (InputPad::state[InputPad::playerPadNum].Gamepad.sThumbRX > 0)
		{
			InputPad::stick[InputPad::playerPadNum][STICK_RIGHT_X] = InputPad::state[InputPad::playerPadNum].Gamepad.sThumbRX - InputPad::stickDeadZone.RIGHT_RIGHT;
		}
		else
		{
			InputPad::stick[InputPad::playerPadNum][STICK_RIGHT_X] = InputPad::state[InputPad::playerPadNum].Gamepad.sThumbRX - InputPad::stickDeadZone.RIGHT_LEFT;
		}
	}
	else		// �X�e�B�b�N�𑀍삵�Ă��Ȃ�
	{
		InputPad::stick[InputPad::playerPadNum][STICK_RIGHT_X] = 0;
	}
	// �E�X�e�B�b�N�̏㉺
	if (InputPad::state[InputPad::playerPadNum].Gamepad.sThumbRY > stickDeadZone.RIGHT_UP
		|| InputPad::state[InputPad::playerPadNum].Gamepad.sThumbRY < stickDeadZone.RIGHT_DOWN)		// �X�e�B�b�N�𑀍삵����
	{
		if (InputPad::state[InputPad::playerPadNum].Gamepad.sThumbRY > 0)
		{
			InputPad::stick[InputPad::playerPadNum][STICK_RIGHT_Y] = InputPad::state[InputPad::playerPadNum].Gamepad.sThumbRY - InputPad::stickDeadZone.RIGHT_UP;
		}
		else
		{
			InputPad::stick[InputPad::playerPadNum][STICK_RIGHT_Y] = InputPad::state[InputPad::playerPadNum].Gamepad.sThumbRY - InputPad::stickDeadZone.RIGHT_DOWN;
		}
	}
	else		// �X�e�B�b�N�𑀍삵�Ă��Ȃ�
	{
		InputPad::stick[InputPad::playerPadNum][STICK_RIGHT_Y] = 0;
	}
}


// �o�C�u���[�V�������s��
void InputPad::Vibration(const unsigned __int8 use_padnum, const int time, const unsigned __int16 rightVib, const unsigned __int16 leftVib)
{
	// �o�C�u���[�V�����l
	InputPad::vibration.wRightMotorSpeed = rightVib;
	InputPad::vibration.wLeftMotorSpeed = leftVib;

	XInputSetState(use_padnum, &InputPad::vibration);		// �o�[�u���[�V�����l��ݒ�
}


// �o�C�u���[�V�������~�߂�
void InputPad::VibrationStop(const unsigned __int8 use_padnum)
{
	// �o�C�u���[�V�����l
	InputPad::vibration.wRightMotorSpeed = 0;				// 0�ɂ���
	InputPad::vibration.wLeftMotorSpeed = 0;				// 0�ɂ���

	XInputSetState(use_padnum, &InputPad::vibration);		// �o�C�u���[�V�����l��ݒ�
}

void InputPad::SetPlayerPadNum(const unsigned __int8 playerPadNum)
{
	InputPad::playerPadNum = playerPadNum;
}


// �R���g���[���[�̌q�����Ă��鐔
const int InputPad::GetPadNum()
{
	return (int)InputPad::controllerNum;
}

// �{�^�����͎擾
const int InputPad::GetPadButtonData(const unsigned __int8 use_padnum, const unsigned __int8 use_button)
{
	return InputPad::button[use_padnum][use_button];
}

// �g���K�[���͎擾
const int InputPad::GetPadTriggerData(const unsigned __int8 use_padnum, const bool use_Is_triggerLeft)
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

// �X�e�B�b�N���͎擾
const int InputPad::GetPadThumbData(const unsigned __int8 use_padnum, const unsigned __int8 use_stick)
{
	return InputPad::stick[use_padnum][use_stick];
}

// 
const short MY_XINPUT::InputPad::GetPadThumbMax(const bool stickLightNow, const bool stickXAxisNow, const bool stickPlusNow)
{
	if (stickLightNow)
	{
		if (stickXAxisNow)
		{
			if (stickPlusNow)
			{
				return MAX_STICK_PLUS - InputPad::stickDeadZone.RIGHT_RIGHT;
			}
			else
			{
				return -(MAX_STICK_MINUS - InputPad::stickDeadZone.RIGHT_LEFT);
			}
		}
		else
		{
			if (stickPlusNow)
			{
				return MAX_STICK_PLUS - InputPad::stickDeadZone.RIGHT_UP;
			}
			else
			{
				return -(MAX_STICK_MINUS - InputPad::stickDeadZone.RIGHT_DOWN);
			}
		}
	}
	else
	{
		if (stickXAxisNow)
		{
			if (stickPlusNow)
			{
				return MAX_STICK_PLUS - InputPad::stickDeadZone.LEFT_RIGHT;
			}
			else
			{
				return -(MAX_STICK_MINUS - InputPad::stickDeadZone.LEFT_LEFT);
			}
		}
		else
		{
			if (stickPlusNow)
			{
				return MAX_STICK_PLUS - InputPad::stickDeadZone.LEFT_UP;
			}
			else
			{
				return -(MAX_STICK_MINUS - InputPad::stickDeadZone.LEFT_DOWN);
			}
		}
	}
}

// �X�e�B�b�N�̃f�b�h�]�[���ݒ�
void InputPad::SetPadDeadZone(const short leftPad_right, const short leftPad_left
	, const short leftPad_up, const short leftPad_down, const short rightPad_right, const short rightPad_left, const short rightPad_up, const short rightPad_down)
{
	InputPad::stickDeadZone.LEFT_RIGHT	 = (leftPad_right	 == XINPUT_GAMEPAD_LEFT_THUMB_DEADZONE)		 ? XINPUT_GAMEPAD_LEFT_THUMB_DEADZONE : leftPad_right;
	InputPad::stickDeadZone.LEFT_LEFT	 = (leftPad_left	 == -XINPUT_GAMEPAD_LEFT_THUMB_DEADZONE)	 ? -XINPUT_GAMEPAD_LEFT_THUMB_DEADZONE : leftPad_left;
	InputPad::stickDeadZone.LEFT_UP		 = (leftPad_up		 == XINPUT_GAMEPAD_LEFT_THUMB_DEADZONE)		 ? XINPUT_GAMEPAD_LEFT_THUMB_DEADZONE : leftPad_up;
	InputPad::stickDeadZone.LEFT_DOWN	 = (leftPad_down	 == -XINPUT_GAMEPAD_LEFT_THUMB_DEADZONE)	 ? -XINPUT_GAMEPAD_LEFT_THUMB_DEADZONE : leftPad_down;
	InputPad::stickDeadZone.RIGHT_RIGHT	 = (rightPad_right	 == XINPUT_GAMEPAD_RIGHT_THUMB_DEADZONE)	 ? XINPUT_GAMEPAD_RIGHT_THUMB_DEADZONE : rightPad_right;
	InputPad::stickDeadZone.RIGHT_LEFT	 = (rightPad_left	 == -XINPUT_GAMEPAD_RIGHT_THUMB_DEADZONE)	 ? -XINPUT_GAMEPAD_RIGHT_THUMB_DEADZONE : rightPad_left;
	InputPad::stickDeadZone.RIGHT_UP	 = (rightPad_up		 == XINPUT_GAMEPAD_RIGHT_THUMB_DEADZONE)	 ? XINPUT_GAMEPAD_RIGHT_THUMB_DEADZONE : rightPad_up;
	InputPad::stickDeadZone.RIGHT_DOWN	 = (rightPad_down	 == -XINPUT_GAMEPAD_RIGHT_THUMB_DEADZONE)	 ? -XINPUT_GAMEPAD_RIGHT_THUMB_DEADZONE : rightPad_down;
}

void MY_XINPUT::InputPad::InitPadDeadZone()
{
	InputPad::stickDeadZone.LEFT_RIGHT	 = XINPUT_GAMEPAD_LEFT_THUMB_DEADZONE;
	InputPad::stickDeadZone.LEFT_LEFT	 = -XINPUT_GAMEPAD_LEFT_THUMB_DEADZONE;
	InputPad::stickDeadZone.LEFT_UP		 = XINPUT_GAMEPAD_LEFT_THUMB_DEADZONE;
	InputPad::stickDeadZone.LEFT_DOWN	 = -XINPUT_GAMEPAD_LEFT_THUMB_DEADZONE;
	InputPad::stickDeadZone.RIGHT_RIGHT	 = XINPUT_GAMEPAD_RIGHT_THUMB_DEADZONE;
	InputPad::stickDeadZone.RIGHT_LEFT	 = -XINPUT_GAMEPAD_RIGHT_THUMB_DEADZONE;
	InputPad::stickDeadZone.RIGHT_UP	 = XINPUT_GAMEPAD_RIGHT_THUMB_DEADZONE;
	InputPad::stickDeadZone.RIGHT_DOWN	 = -XINPUT_GAMEPAD_RIGHT_THUMB_DEADZONE;
}
