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

		printfDx("%s\n", "Debug");
	}

	// 削除
	delete camera;
	delete character;

	DxLib::DxLib_End();		// DXライブラリの後始末

	return 0;
}