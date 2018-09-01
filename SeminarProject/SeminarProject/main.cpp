//#include "MainMove1.hpp"
//#include "LoadThread.hpp"
//#include "BaseMove.hpp"
//#include "LoadScreen.hpp"
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

	SetWindowText("SeminarProject");	// ���C���E�C���h�E�̃E�C���h�E�^�C�g����ύX����

	SetBackgroundColor(128, 128, 128);

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

	SetDrawScreen(DX_SCREEN_BACK);	// �w�i�`��

	// �R���g���[���[�ƃL�[�{�[�h�̏�����
	MYINPUTPAD::InputPad::InputPad();
	MYINPUTPAD::InputPad::Update();
#ifdef _DEBUG
	KeyData::UpDate();
#endif

	// new
	Manager* manager = new Manager();
	/*LoadThread* loadThread = new LoadThread();
	BaseMove* move1 = NULL;
	LoadScreen* loadScreen = new LoadScreen();

	const int max1 = 3;
	std::string move1str[max1];
	ELOADFILE load1[max1];
	move1str[0] = "media\\�X�e�[�W���f��\\move1_graphic.myn";
	move1str[1] = "media\\�X�e�[�W���f��\\move1_hantei.myn";
	move1str[2] = "media\\CLPH\\motion\\CLPH_motionALL.myn";
	load1[0] = ELOADFILE::mv1model;
	load1[1] = ELOADFILE::mv1model;
	load1[2] = ELOADFILE::mv1model;
	bool flag = false;*/

	/*std::vector<int> file(3);
	LoadFile::MyLoad("media\\�X�e�[�W���f��\\move1_graphic.myn", file[0], ELOADFILE::mv1model);
	LoadFile::MyLoad("media\\�X�e�[�W���f��\\move1_hantei.myn", file[1], ELOADFILE::mv1model);
	LoadFile::MyLoad("media\\CLPH\\motion\\CLPH_motionALL.myn", file[2], ELOADFILE::mv1model);*/

	//BaseMove* move1 = new MainMove1(file);

	// �ŏ��ɃR���g���[���[��ݒ肷�邽�߂̊m�F�R�}���h
	bool firstControll = false;						// �R���g���[���[��������ĂȂ��̂ŃQ�[�����N�����Ȃ��悤
	unsigned __int8 controllNumber = 5;				// �����ꂽ�R���g���[���[�̔ԍ�
	int controllCount = 0;							// �R�}���h�Ɋւ��鎞��
	bool noTouch = true;							// �R�}���h��������Ȃ����Ԍo�ߎ���ōċN���𑣂��悤����
	const int COUNT = 600;							// �R�}���h���Ԃ̐��l

	// �Q�[���̊j
	while (ScreenFlip() == 0 && ProcessMessage() == 0 && ClearDrawScreen() == 0 && CheckHitKey(KEY_INPUT_ESCAPE) == 0)
	{
		MYINPUTPAD::InputPad::Update();

		// �ڑ�������̏ꍇ�͊m�F���Ȃ�
		if (MYINPUTPAD::InputPad::GetPadNum() == 1)
		{
			controllNumber = 0;
			firstControll = true;
		}
		// �R���g���[���[���Q�ȏ�̎�
		if (!firstControll)
		{
			// �͈͊O�ɓ����Ƃ����܂�
			if (controllNumber == 5)
			{
				controllCount++;
				DrawFormatString(winWidth / 2, winHeight / 2, GetColor(255, 255, 255), "�R���g���[���[��A�{�^���������Ă��������B\n������R���g���[���[�Ƃ��ĔF�؂��܂��B\n");
				if (MYINPUTPAD::InputPad::GetPadButtonData(0, MYINPUTPAD::XINPUT_PAD::BUTTON_A) == 1)		// �PP�����͂��ꂽ
				{
					controllNumber = 0;
					controllCount = 0;
				}
				if (MYINPUTPAD::InputPad::GetPadButtonData(1, MYINPUTPAD::XINPUT_PAD::BUTTON_A) == 1)		// �QP�����͂��ꂽ
				{
					controllNumber = 1;
					controllCount = 0;
				}
				if (MYINPUTPAD::InputPad::GetPadButtonData(2, MYINPUTPAD::XINPUT_PAD::BUTTON_A) == 1)		// �RP�����͂��ꂽ
				{
					controllNumber = 2;
					controllCount = 0;
				}
				if (MYINPUTPAD::InputPad::GetPadButtonData(3, MYINPUTPAD::XINPUT_PAD::BUTTON_A) == 1)		// �SP�����͂��ꂽ
				{
					controllNumber = 3;
					controllCount = 0;
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
			manager->Update(controllNumber);
		}
	}

	// �폜
	delete manager;

	DxLib::DxLib_End();		// DX���C�u�����̌�n��

	return 0;
}