#include "Character.hpp"
#include "Camera.hpp"

int WINAPI WinMain(HINSTANCE hInstance, HINSTANCE hPrevInstance, LPSTR lpCmdLine, int nCmdShow)
{
#ifdef _DEBUG
	SetOutApplicationLogValidFlag(TRUE);	// ログテキスト出力する
#elif NDEBUG
	SetOutApplicationLogValidFlag(FALSE);	// ログテキスト出力しない
#endif


	SetWindowText("SeminarProject");	// メインウインドウのウインドウタイトルを変更する

	//SetBackgroundColor(255, 255, 255);

	SetUseDirect3DVersion(DX_DIRECT3D_11);			// Direct3D11を使用する

	ChangeWindowMode(TRUE);			// ウィンドウズモードにさせるかどうか

	SetEnableXAudioFlag(TRUE);			// XAudioを使用するようにする

	SetGraphMode(1920, 1080, 32);					// 1920x1080x32bit

	if (DxLib_Init() == -1)		// ＤＸライブラリ初期化処理
	{
		return -1;			// エラーが起きたら直ちに終了
	}

	SetDrawScreen(DX_SCREEN_BACK);	// 背景描画

	SetBackgroundColor(128, 128, 128);	// 背景の色を灰色にする

	// 床に線を引くためのもの----------------------------------------------
	int i;
	VECTOR Pos1;
	VECTOR Pos2;
	float LINE_AREA_SIZE;
	int LINE_NUM;
	LINE_AREA_SIZE = 2000.0f;
	LINE_NUM = 50;
	// ---------------------------------------------------------------------

	// コントローラーとキーボードの初期化
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

		// 床に線を引く--------------------------------------------------------
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

	// 削除
	delete camera;
	delete character;

	DxLib::DxLib_End();		// DXライブラリの後始末

	return 0;
}