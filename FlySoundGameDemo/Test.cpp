#include "DxLib.h"

int WINAPI WinMain(HINSTANCE hInstance, HINSTANCE hPrevInstance, LPSTR lpCmdLine, int nCmdShow)
{
	int ModelHandle, AttachIndex;
	float TotalTime, PlayTime;

	ChangeWindowMode(true);

	// �c�w���C�u�����̏�����
	if (DxLib_Init() < 0)
	{
		// �G���[�����������璼���ɏI��
		return -1;
	}

	// �R�c���f���̓ǂݍ���
	ModelHandle = MV1LoadModel("media\\�P_�ϐg��\\���[�V����\\TestYvee.fbx");
	DrawFormatString(0, 0, 255, "%d", ModelHandle);

	// �`���𗠉�ʂɕύX
	SetDrawScreen(DX_SCREEN_BACK);

	// ��ʂɉf��ʒu�ɂR�c���f�����ړ�
	MV1SetPosition(ModelHandle, VGet(320.0f, 70.0f, -70.0f));

	// �R�c���f���̂O�Ԗڂ̃A�j���[�V�������A�^�b�`����
	AttachIndex = MV1AttachAnim(ModelHandle, 0, -1, FALSE);

	// �A�^�b�`�����A�j���[�V�����̑��Đ����Ԃ��擾����
	TotalTime = MV1GetAttachAnimTotalTime(ModelHandle, AttachIndex);

	// �Đ����Ԃ̏�����
	PlayTime = 0.0f;

	// �����L�[��������邩�E�C���h�E��������܂Ń��[�v
	while (ProcessMessage() == 0 && CheckHitKeyAll() == 0)
	{
		// ��ʂ��N���A
		ClearDrawScreen();

		// �Đ����Ԃ�i�߂�
		PlayTime += 0.5f;

		// �Đ����Ԃ��A�j���[�V�����̑��Đ����ԂɒB������Đ����Ԃ��O�ɖ߂�
		if (PlayTime >= TotalTime)
		{
			PlayTime = 0.0f;
		}

		// �Đ����Ԃ��Z�b�g����
		MV1SetAttachAnimTime(ModelHandle, AttachIndex, PlayTime);

		// �R�c���f���̕`��
		MV1DrawModel(ModelHandle);

		// ����ʂ̓��e��\��ʂɔ��f
		ScreenFlip();
	}

	// ���f���n���h���̍폜
	MV1DeleteModel(ModelHandle);

	// �c�w���C�u�����̌�n��
	DxLib_End();

	// �\�t�g�̏I��
	return 0;
}