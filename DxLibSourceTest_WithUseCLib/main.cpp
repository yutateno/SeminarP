#include "Character.hpp"
#include "Camera.hpp"

int WINAPI WinMain(HINSTANCE hInstance, HINSTANCE hPrevInstance, LPSTR lpCmdLine, int nCmdShow)
{
#ifdef _DEBUG
	SetOutApplicationLogValidFlag(TRUE);	// ���O�e�L�X�g�o�͂���
#elif NDEBUG
	SetOutApplicationLogValidFlag(FALSE);	// ���O�e�L�X�g�o�͂��Ȃ�
#endif


	SetWindowText("SeminarProject");	// ���C���E�C���h�E�̃E�C���h�E�^�C�g����ύX����

	//SetBackgroundColor(255, 255, 255);

	SetUseDirect3DVersion(DX_DIRECT3D_11);			// Direct3D11���g�p����

	ChangeWindowMode(TRUE);			// �E�B���h�E�Y���[�h�ɂ����邩�ǂ���

	SetEnableXAudioFlag(TRUE);			// XAudio���g�p����悤�ɂ���

	SetGraphMode(1920, 1080, 32);					// 1920x1080x32bit

	if (DxLib_Init() == -1)		// �c�w���C�u��������������
	{
		return -1;			// �G���[���N�����璼���ɏI��
	}

	SetDrawScreen(DX_SCREEN_BACK);	// �w�i�`��

	SetBackgroundColor(128, 128, 128);	// �w�i�̐F���D�F�ɂ���

	// ���ɐ����������߂̂���----------------------------------------------
	int i;
	VECTOR Pos1;
	VECTOR Pos2;
	float LINE_AREA_SIZE;
	int LINE_NUM;
	LINE_AREA_SIZE = 2000.0f;
	LINE_NUM = 50;
	// ---------------------------------------------------------------------

	// �R���g���[���[�ƃL�[�{�[�h�̏�����
	MYINPUTPAD::InputPad::InputPad();
	MYINPUTPAD::InputPad::Update();
	KeyData::UpDate();

	// new
	Character* character = new Character();
	Camera* camera = new Camera(character->GetArea());

	while (ScreenFlip() == 0 && ProcessMessage() == 0 && ClearDrawScreen() == 0 && KeyData::CheckEnd() != 0)
	{
		KeyData::UpDate();
		MYINPUTPAD::InputPad::Update();

		character->Process();
		character->Draw();
		camera->Process(character->GetArea());

		// ���ɐ�������--------------------------------------------------------
		SetUseZBufferFlag(TRUE);
		Pos1 = VGet(-LINE_AREA_SIZE / 2.0f, 0.0f, -LINE_AREA_SIZE / 2.0f);
		Pos2 = VGet(-LINE_AREA_SIZE / 2.0f, 0.0f, LINE_AREA_SIZE / 2.0f);
		for (i = 0; i <= LINE_NUM; i++)
		{
			DrawLine3D(Pos1, Pos2, GetColor(255, 255, 255));
			Pos1.x += LINE_AREA_SIZE / LINE_NUM;
			Pos2.x += LINE_AREA_SIZE / LINE_NUM;
		}

		Pos1 = VGet(-LINE_AREA_SIZE / 2.0f, 0.0f, -LINE_AREA_SIZE / 2.0f);
		Pos2 = VGet(LINE_AREA_SIZE / 2.0f, 0.0f, -LINE_AREA_SIZE / 2.0f);
		for (i = 0; i < LINE_NUM; i++)
		{
			DrawLine3D(Pos1, Pos2, GetColor(255, 255, 255));
			Pos1.z += LINE_AREA_SIZE / LINE_NUM;
			Pos2.z += LINE_AREA_SIZE / LINE_NUM;
		}
		SetUseZBufferFlag(FALSE);
		// ---------------------------------------------------------------------

		printfDx("%s\n", "Debug");
	}

	// �폜
	delete camera;
	delete character;

	DxLib::DxLib_End();		// DX���C�u�����̌�n��

	return 0;
}