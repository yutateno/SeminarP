#include "Manager.hpp"

// �E�B���h�E�T�C�Y
int winWidth = 1920;
int winHeight = 1080;

void ProjectInit()
{
#ifdef _DEBUG
	SetOutApplicationLogValidFlag(TRUE);	// ���O�e�L�X�g�o�͂���
#elif NDEBUG
	SetOutApplicationLogValidFlag(FALSE);	// ���O�e�L�X�g�o�͂��Ȃ�
#endif

	SetWindowText("Re.Gleam");	// ���C���E�C���h�E�̃E�C���h�E�^�C�g����ύX����

	SetBackgroundColor(255, 255, 255);

	SetUseDirect3DVersion(DX_DIRECT3D_11);			// Direct3D11���g�p����

	ChangeWindowMode(TRUE);			// �E�B���h�E�Y���[�h�ɂ����邩�ǂ���

	SetEnableXAudioFlag(TRUE);			// XAudio���g�p����悤�ɂ���

	SetUseLarge3DPositionSupport(TRUE);		// ����ȍ��W�l���T�|�[�g

	SetGraphMode(winWidth, winHeight, 32);					// 1920x1080x32bit
}

int WINAPI WinMain(HINSTANCE hInstance, HINSTANCE hPrevInstance, LPSTR lpCmdLine, int nCmdShow)
{
	ProjectInit();		// DX���C�u�����������O������

	if (DxLib_Init() == -1)		// �c�w���C�u��������������
	{
		return -1;			// �G���[���N�����璼���ɏI��
	}

	SetAlwaysRunFlag(TRUE);

	SetDrawScreen(DX_SCREEN_BACK);	// �w�i�`��

	// �R���g���[���[�ƃL�[�{�[�h�̏�����
	MY_XINPUT::InputPad::InputPad();
	MY_XINPUT::InputPad::FirstUpdate();
#ifdef _DEBUG
	KeyData::UpDate();
#endif

	// new
	Manager* manager = new Manager();

	// �ŏ��ɃR���g���[���[��ݒ肷�邽�߂̊m�F�R�}���h
	bool firstControll = false;						// �R���g���[���[��������ĂȂ��̂ŃQ�[�����N�����Ȃ��悤
	unsigned __int8 controllNumber = 5;				// �\���p�̃R���g���[���[
	int controllCount = 0;							// �R�}���h�Ɋւ��鎞��
	bool noTouch = true;							// �R�}���h��������Ȃ����Ԍo�ߎ���ōċN���𑣂��悤����
	const int COUNT = 600;							// �R�}���h���Ԃ̐��l

	// �Q�[���̊j
	while (ScreenFlip() == 0 && ProcessMessage() == 0 && ClearDrawScreen() == 0 && CheckHitKey(KEY_INPUT_ESCAPE) == 0)
	{
		// �ڑ�������̏ꍇ�͊m�F���Ȃ�
		if (MY_XINPUT::InputPad::GetPadNum() == 1)
		{
			MY_XINPUT::InputPad::SetPlayerPadNum(MY_XINPUT::NUM01);
			controllNumber = MY_XINPUT::NUM01;
			firstControll = true;
		}
		// �R���g���[���[���Q�ȏ�̎�
		if (!firstControll)
		{
			MY_XINPUT::InputPad::FirstUpdate();

			// �͈͊O�ɓ����Ƃ����܂�
			if (controllNumber == 5)
			{
				controllCount++;
				DrawFormatString(winWidth / 2, winHeight / 2, GetColor(255, 255, 255), "�R���g���[���[��A�{�^���������Ă��������B\n������R���g���[���[�Ƃ��ĔF�؂��܂��B\n");
				if (controllCount >= 10)
				{
					if (MY_XINPUT::InputPad::GetPadButtonData(0, MY_XINPUT::BUTTON_A) == 1)		// �PP�����͂��ꂽ
					{
						MY_XINPUT::InputPad::SetPlayerPadNum(MY_XINPUT::NUM01);
						controllCount = 0;
						controllNumber = MY_XINPUT::NUM01;
					}
					if (MY_XINPUT::InputPad::GetPadButtonData(1, MY_XINPUT::BUTTON_A) == 1)		// �QP�����͂��ꂽ
					{
						MY_XINPUT::InputPad::SetPlayerPadNum(MY_XINPUT::NUM02);
						controllCount = 0;
						controllNumber = MY_XINPUT::NUM02;
					}
					if (MY_XINPUT::InputPad::GetPadButtonData(2, MY_XINPUT::BUTTON_A) == 1)		// �RP�����͂��ꂽ
					{
						MY_XINPUT::InputPad::SetPlayerPadNum(MY_XINPUT::NUM03);
						controllCount = 0;
						controllNumber = MY_XINPUT::NUM03;
					}
					if (MY_XINPUT::InputPad::GetPadButtonData(3, MY_XINPUT::BUTTON_A) == 1)		// �SP�����͂��ꂽ
					{
						MY_XINPUT::InputPad::SetPlayerPadNum(MY_XINPUT::NUM04);
						controllCount = 0;
						controllNumber = MY_XINPUT::NUM04;
					}
				}

				// ���͂���Ȃ����Ԍo�߂œ�����^����
				if (controllCount >= COUNT && controllCount < COUNT + 400)
				{
					DrawFormatString(winWidth / 2, (winHeight / 2) + 100, GetColor(255, 255, 255), "���͂���莞�Ԋm�F�ł��܂���B�ċN�����Ă݂Ă��������B\n");
				}
				else if (controllCount >= COUNT + 400 && controllCount < COUNT + 550)		// ���������肪����Ɣ��f���ďI��������
				{
					DrawFormatString(winWidth / 2, (winHeight / 2) + 100, GetColor(255, 255, 255), "��肪�������Ă�Ɣ��f���A�Q�[�����I�����܂��B\n");
				}
				else if (controllCount >= COUNT + 550)
				{
					break;
				}
			}
			else
			{
				controllCount++;
				DrawFormatString(winWidth / 2, winHeight / 2, GetColor(255, 255, 255), "�R���g���[���[�i���o�[�F%d ���m�F���܂����B�Q�[�����J�n���܂��B\n", (controllNumber + 1));
				if (controllCount >= 100)
				{
					firstControll = true;
				}
			}
		}
		else
		{
#ifdef _DEBUG
			KeyData::UpDate();
#endif
			MY_XINPUT::InputPad::EverUpdate();

			manager->Update(controllNumber);
		}
	}

	// �폜
	delete manager;

	DxLib::DxLib_End();		// DX���C�u�����̌�n��

	return 0;
}