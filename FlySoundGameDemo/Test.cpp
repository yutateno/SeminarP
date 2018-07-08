#include "DxLib.h"

int WINAPI WinMain(HINSTANCE hInstance, HINSTANCE hPrevInstance, LPSTR lpCmdLine, int nCmdShow)
{
	int ModelHandle, AttachIndex;
	float TotalTime, PlayTime;
	int Direct3DVersion;

	ChangeWindowMode(true);

	// �c�w���C�u�����̏�����
	if (DxLib_Init() < 0)
	{
		// �G���[�����������璼���ɏI��
		return -1;
	}

	// �R�c���f���̓ǂݍ���
	ModelHandle = MV1LoadModel("media\\�P_�ϐg��\\���[�V����\\TestYvee.fbx");

	// �`���𗠉�ʂɕύX
	SetDrawScreen(DX_SCREEN_BACK);
	
	// �g�p���Ă��� Direct3D �̃o�[�W�������擾
	Direct3DVersion = GetUseDirect3DVersion();

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

		DrawFormatString(0, 100, 255, "%d", ModelHandle);

		// �g�p�o�[�W������ Direct3D 9Ex ���ǂ������`�F�b�N
		if (Direct3DVersion == DX_DIRECT3D_9EX)
		{
			DrawString(0, 0, "Direct3D 9Ex ���g�p���Ă��܂�", GetColor(255, 255, 255));
		}
		else if (Direct3DVersion == DX_DIRECT3D_11)
		{
			DrawString(0, 0, "Direct3D 11 ���g�p���Ă��܂�", GetColor(255, 255, 255));
		}
		else
		{
			DrawString(0, 0, "Direct3D  ���g�p����", GetColor(255, 255, 255));
		}

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