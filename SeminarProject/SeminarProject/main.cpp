#include "Manager.hpp"

// ウィンドウサイズ
int winWidth = 1920;
int winHeight = 1080;

void ProjectInit()
{
#ifdef _DEBUG
	SetOutApplicationLogValidFlag(TRUE);	// ログテキスト出力する
#elif NDEBUG
	SetOutApplicationLogValidFlag(FALSE);	// ログテキスト出力しない
#endif

	SetWindowText("Re.Gleam");	// メインウインドウのウインドウタイトルを変更する

	SetBackgroundColor(255, 255, 255);

	SetUseDirect3DVersion(DX_DIRECT3D_11);			// Direct3D11を使用する

	ChangeWindowMode(TRUE);			// ウィンドウズモードにさせるかどうか

	SetEnableXAudioFlag(TRUE);			// XAudioを使用するようにする

	SetUseLarge3DPositionSupport(TRUE);		// 巨大な座標値をサポート

	SetGraphMode(winWidth, winHeight, 32);					// 1920x1080x32bit
}

int WINAPI WinMain(HINSTANCE hInstance, HINSTANCE hPrevInstance, LPSTR lpCmdLine, int nCmdShow)
{
	ProjectInit();		// DXライブラリ初期化前初期化

	if (DxLib_Init() == -1)		// ＤＸライブラリ初期化処理
	{
		return -1;			// エラーが起きたら直ちに終了
	}

	SetAlwaysRunFlag(TRUE);

	SetDrawScreen(DX_SCREEN_BACK);	// 背景描画

	// コントローラーとキーボードの初期化
	MY_XINPUT::InputPad::InputPad();
	MY_XINPUT::InputPad::FirstUpdate();
#ifdef _DEBUG
	KeyData::UpDate();
#endif

	// new
	Manager* manager = new Manager();

	// 最初にコントローラーを設定するための確認コマンド
	bool firstControll = false;						// コントローラーが押されてないのでゲームを起動しないよう
	unsigned __int8 controllNumber = 5;				// 表示用のコントローラー
	int controllCount = 0;							// コマンドに関する時間
	bool noTouch = true;							// コマンドを押されない時間経過次第で再起動を促すよう処理
	const int COUNT = 600;							// コマンド時間の数値

	// ゲームの核
	while (ScreenFlip() == 0 && ProcessMessage() == 0 && ClearDrawScreen() == 0 && CheckHitKey(KEY_INPUT_ESCAPE) == 0)
	{
		// 接続数が一つの場合は確認しない
		if (MY_XINPUT::InputPad::GetPadNum() == 1)
		{
			MY_XINPUT::InputPad::SetPlayerPadNum(MY_XINPUT::NUM01);
			controllNumber = MY_XINPUT::NUM01;
			firstControll = true;
		}
		// コントローラーが２つ以上の時
		if (!firstControll)
		{
			MY_XINPUT::InputPad::FirstUpdate();

			// 範囲外に投げといたまま
			if (controllNumber == 5)
			{
				controllCount++;
				DrawFormatString(winWidth / 2, winHeight / 2, GetColor(255, 255, 255), "コントローラーのAボタンを押してください。\nそれをコントローラーとして認証します。\n");
				if (controllCount >= 10)
				{
					if (MY_XINPUT::InputPad::GetPadButtonData(0, MY_XINPUT::BUTTON_A) == 1)		// １Pが入力された
					{
						MY_XINPUT::InputPad::SetPlayerPadNum(MY_XINPUT::NUM01);
						controllCount = 0;
						controllNumber = MY_XINPUT::NUM01;
					}
					if (MY_XINPUT::InputPad::GetPadButtonData(1, MY_XINPUT::BUTTON_A) == 1)		// ２Pが入力された
					{
						MY_XINPUT::InputPad::SetPlayerPadNum(MY_XINPUT::NUM02);
						controllCount = 0;
						controllNumber = MY_XINPUT::NUM02;
					}
					if (MY_XINPUT::InputPad::GetPadButtonData(2, MY_XINPUT::BUTTON_A) == 1)		// ３Pが入力された
					{
						MY_XINPUT::InputPad::SetPlayerPadNum(MY_XINPUT::NUM03);
						controllCount = 0;
						controllNumber = MY_XINPUT::NUM03;
					}
					if (MY_XINPUT::InputPad::GetPadButtonData(3, MY_XINPUT::BUTTON_A) == 1)		// ４Pが入力された
					{
						MY_XINPUT::InputPad::SetPlayerPadNum(MY_XINPUT::NUM04);
						controllCount = 0;
						controllNumber = MY_XINPUT::NUM04;
					}
				}

				// 入力されない時間経過で動きを与える
				if (controllCount >= COUNT && controllCount < COUNT + 400)
				{
					DrawFormatString(winWidth / 2, (winHeight / 2) + 100, GetColor(255, 255, 255), "入力を一定時間確認できません。再起動してみてください。\n");
				}
				else if (controllCount >= COUNT + 400 && controllCount < COUNT + 550)		// 何かしら問題があると判断して終了させる
				{
					DrawFormatString(winWidth / 2, (winHeight / 2) + 100, GetColor(255, 255, 255), "問題が発生してると判断し、ゲームを終了します。\n");
				}
				else if (controllCount >= COUNT + 550)
				{
					break;
				}
			}
			else
			{
				controllCount++;
				DrawFormatString(winWidth / 2, winHeight / 2, GetColor(255, 255, 255), "コントローラーナンバー：%d を確認しました。ゲームを開始します。\n", (controllNumber + 1));
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

	// 削除
	delete manager;

	DxLib::DxLib_End();		// DXライブラリの後始末

	return 0;
}