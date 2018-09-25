#pragma once
#ifndef _MY_INPUTPAD_H
#define _MY_INPUTPAD_H


#include <Windows.h>
#include <math.h>


namespace MY_XINPUT	// XINPUT_STATE�������܂��ƂȂ�̂ňꊇ�X�R�[�v����
{

#ifndef _COMPILE_SLOPE					// �C�������������`��������
#define _COMPILE_SLOPE
#include <Xinput.h>

#pragma comment(lib, "xinput.lib")

#endif // !_COMPILE_SLOPE


	namespace 
	{
		// �ԍ�
		const unsigned __int8 NUM01 = 0;
		const unsigned __int8 NUM02 = 1;
		const unsigned __int8 NUM03 = 2;
		const unsigned __int8 NUM04 = 3;


		// �{�^��
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


		// �g���K
		const bool TRIGGER_RT = 0;
		const bool TRIGGER_LT = 1;

		const unsigned __int8 SHOULDER_LB = 8;
		const unsigned __int8 SHOULDER_RB = 9;


		// �X�e�B�b�N
		const unsigned __int8 STICK_RIGHT_X	 = 0;
		const unsigned __int8 STICK_RIGHT_Y	 = 1;
		const unsigned __int8 STICK_LEFT_X	 = 2;
		const unsigned __int8 STICK_LEFT_Y	 = 3;

		const int MAX_STICK_PLUS = 32767;
		const int MAX_STICK_MINUS = -32768;


		// �o�C�u���[�V����
		const unsigned __int16 VIB_MAX = 65535;
	}

	struct STICK_DEADZONE
	{
		// ���X�e�B�b�N�̍��E
		short LEFT_RIGHT = XINPUT_GAMEPAD_LEFT_THUMB_DEADZONE;
		short LEFT_LEFT	 = -XINPUT_GAMEPAD_LEFT_THUMB_DEADZONE;


		// ���X�e�B�b�N�̏㉺
		short LEFT_UP	 = XINPUT_GAMEPAD_LEFT_THUMB_DEADZONE;
		short LEFT_DOWN	 = -XINPUT_GAMEPAD_LEFT_THUMB_DEADZONE;


		// �E�X�e�B�b�N�̍��E
		short RIGHT_RIGHT	 = XINPUT_GAMEPAD_RIGHT_THUMB_DEADZONE;
		short RIGHT_LEFT	 = -XINPUT_GAMEPAD_RIGHT_THUMB_DEADZONE;


		// �E�X�e�B�b�N�̏㉺
		short RIGHT_UP	 = XINPUT_GAMEPAD_RIGHT_THUMB_DEADZONE;
		short RIGHT_DOWN = -XINPUT_GAMEPAD_RIGHT_THUMB_DEADZONE;
	};


	class InputPad
	{
	private:
		static unsigned __int8 controllerNum;		// �ڑ����Ă���ő�̌�
		static bool setControll[4];					// �g���鑀��̔ԍ�


		static unsigned __int8 playerPadNum;		// �v���C���[���g���R���g���[���[


		static int button[4][16];					// wButton�̑Ή�
		static int stick[4][4];						// stick�̑Ή�(��������thumb)


		static XINPUT_STATE state[4];				// xinput�̒��g


		static STICK_DEADZONE stickDeadZone;		// �X�e�B�b�N�̃f�b�h�]�[���l


		static XINPUT_VIBRATION vibration;			// �o�C�u���[�V�����̍\����
		

	public:
		InputPad();			// �R���X�g���N�^
		~InputPad();		// �f�X�g���N�^


		static void FirstUpdate();			// �Q�[���J�n�O����X�V
		static void EverUpdate();			// �Q�[���J�n�㑀��X�V


		static void Vibration(const unsigned __int8 use_padnum, const int time = 0
			, const unsigned __int16 rightVib = VIB_MAX, const unsigned __int16 leftVib = VIB_MAX);		// �o�C�u���[�V�������s��

		static void VibrationStop(const unsigned __int8 use_padnum);								// �o�[�u���[�V�������~�߂�


		static void SetPlayerPadNum(const unsigned __int8 playerPadNum);					// �v���C���[�̔ԍ���ݒ�


		static void SetPadDeadZone(const short leftPad_right = XINPUT_GAMEPAD_LEFT_THUMB_DEADZONE, const short leftPad_left = -XINPUT_GAMEPAD_LEFT_THUMB_DEADZONE
			, const short leftPad_up	 = XINPUT_GAMEPAD_LEFT_THUMB_DEADZONE,	 const short leftPad_down = -XINPUT_GAMEPAD_LEFT_THUMB_DEADZONE
			, const short rightPad_right = XINPUT_GAMEPAD_RIGHT_THUMB_DEADZONE,	 const short rightPad_left = -XINPUT_GAMEPAD_RIGHT_THUMB_DEADZONE
			, const short rightPad_up	 = XINPUT_GAMEPAD_RIGHT_THUMB_DEADZONE,	 const short rightPad_down = -XINPUT_GAMEPAD_RIGHT_THUMB_DEADZONE);		// �f�b�h�]�[���̐ݒ�l�ύX  // (������x�y�������̂ňꉞ�f�t�H���g�������p)

		static void InitPadDeadZone();			// �f�b�h�]�[���������l�ɖ߂�


		static const int GetPadNum();																			// �R���g���[���̐�
		static const int GetPadButtonData(const unsigned __int8 use_padnum, const unsigned __int8 use_button);			// �R���g���[���̃{�^������
		static const int GetPadTriggerData(const unsigned __int8 use_padnum, const bool use_Is_triggerLeft);				// �R���g���[���̃g���K�[����
		static const int GetPadThumbData(const unsigned __int8 use_padnum, const unsigned __int8 use_stick);				// �R���g���[���̃X�e�B�b�N����
		static const short GetPadThumbMax(const bool stickLightNow, const bool stickXAxisNow, const bool stickPlusNow);
	};
}

#endif // !_MY_INPUTPAD_H