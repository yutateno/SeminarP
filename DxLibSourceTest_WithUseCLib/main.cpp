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
	int model = MV1LoadModel("media\\�P_�ϐg��\\���[�V����\\attack.fbx");

	// ��ʂɉf��ʒu�ɂR�c���f�����ړ�
	MV1SetPosition(model, VGet(320.0f, 50.0f, 30.0f));

	// �R�c���f���̂O�Ԗڂ̃A�j���[�V�������A�^�b�`����
	int AttachIndex = MV1AttachAnim(model, 0, -1, FALSE);

	// �A�^�b�`�����A�j���[�V�����̑��Đ����Ԃ��擾����
	float TotalTime = MV1GetAttachAnimTotalTime(model, AttachIndex);

	// �Đ����Ԃ̏�����
	float PlayTime = 0.0f;

	while (ScreenFlip() == 0 && ProcessMessage() == 0 && ClearDrawScreen() == 0 && KeyData::CheckEnd() != 0)
	{
		KeyData::UpDate();
		MYINPUTPAD::Input::Update();

		// �Đ����Ԃ�i�߂�
		PlayTime += 0.5f;

		// �Đ����Ԃ��A�j���[�V�����̑��Đ����ԂɒB������Đ����Ԃ��O�ɖ߂�
		if (PlayTime >= TotalTime)
		{
			PlayTime = 0.0f;
		}

		// �Đ����Ԃ��Z�b�g����
		MV1SetAttachAnimTime(model, AttachIndex, PlayTime);

		// �R�c���f���̕`��
		MV1DrawModel(model);
	}

	// ���f���n���h���̍폜
	MV1DeleteModel(model);

	DxLib::DxLib_End();

	return 0;
}