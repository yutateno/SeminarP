#include "DxLib.h"
#include "InputPad.hpp"
#include "InputKey.hpp"

int WINAPI WinMain(HINSTANCE hInstance, HINSTANCE hPrevInstance, LPSTR lpCmdLine, int nCmdShow)
{
#ifdef _DEBUG
	SetOutApplicationLogValidFlag(TRUE);	// ���O�e�L�X�g�o�͂���
#elif NDEBUG
	SetOutApplicationLogValidFlag(FALSE);	// ���O�e�L�X�g�o�͂��Ȃ�
#endif


	SetWindowText("SeminarProject");	// ���C���E�C���h�E�̃E�C���h�E�^�C�g����ύX����
	
	ChangeWindowMode(TRUE);			// �E�B���h�E�Y���[�h�ɂ����邩�ǂ���

	SetGraphMode(640, 480, 32);					// �摜�ɍ��킹�ĉ�ʃT�C�Y��ύX

	if (DxLib_Init() == -1)		// �c�w���C�u��������������
	{
		return -1;			// �G���[���N�����璼���ɏI��
	}

	SetDrawScreen(DX_SCREEN_BACK);	// �w�i�`��

	MYINPUTPAD::Input::Input();
	MYINPUTPAD::Input::Update();
	KeyData::UpDate();

	while (ScreenFlip() == 0 && ProcessMessage() == 0 && ClearDrawScreen() == 0 && KeyData::CheckEnd() != 0)
	{
		KeyData::UpDate();
		MYINPUTPAD::Input::Update();
	}
	
	DxLib::DxLib_End();

	return 0;
}