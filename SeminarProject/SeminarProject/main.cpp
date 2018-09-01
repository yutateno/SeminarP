//#include "MainMove1.hpp"
//#include "LoadThread.hpp"
//#include "BaseMove.hpp"
//#include "LoadScreen.hpp"
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

	SetWindowText("SeminarProject");	// メインウインドウのウインドウタイトルを変更する

	SetBackgroundColor(128, 128, 128);

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

	SetDrawScreen(DX_SCREEN_BACK);	// 背景描画

	// コントローラーとキーボードの初期化
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
	move1str[0] = "media\\ステージモデル\\move1_graphic.myn";
	move1str[1] = "media\\ステージモデル\\move1_hantei.myn";
	move1str[2] = "media\\CLPH\\motion\\CLPH_motionALL.myn";
	load1[0] = ELOADFILE::mv1model;
	load1[1] = ELOADFILE::mv1model;
	load1[2] = ELOADFILE::mv1model;
	bool flag = false;*/

	/*std::vector<int> file(3);
	LoadFile::MyLoad("media\\ステージモデル\\move1_graphic.myn", file[0], ELOADFILE::mv1model);
	LoadFile::MyLoad("media\\ステージモデル\\move1_hantei.myn", file[1], ELOADFILE::mv1model);
	LoadFile::MyLoad("media\\CLPH\\motion\\CLPH_motionALL.myn", file[2], ELOADFILE::mv1model);*/

	//BaseMove* move1 = new MainMove1(file);

	// 最初にコントローラーを設定するための確認コマンド
	bool firstControll = false;						// コントローラーが押されてないのでゲームを起動しないよう
	unsigned __int8 controllNumber = 5;				// 押されたコントローラーの番号
	int controllCount = 0;							// コマンドに関する時間
	bool noTouch = true;							// コマンドを押されない時間経過次第で再起動を促すよう処理
	const int COUNT = 600;							// コマンド時間の数値

	// ゲームの核
	while (ScreenFlip() == 0 && ProcessMessage() == 0 && ClearDrawScreen() == 0 && CheckHitKey(KEY_INPUT_ESCAPE) == 0)
	{
		MYINPUTPAD::InputPad::Update();

		// 接続数が一つの場合は確認しない
		if (MYINPUTPAD::InputPad::GetPadNum() == 1)
		{
			controllNumber = 0;
			firstControll = true;
		}
		// コントローラーが２つ以上の時
		if (!firstControll)
		{
			// 範囲外に投げといたまま
			if (controllNumber == 5)
			{
				controllCount++;
				DrawFormatString(winWidth / 2, winHeight / 2, GetColor(255, 255, 255), "コントローラーのAボタンを押してください。\nそれをコントローラーとして認証します。\n");
				if (MYINPUTPAD::InputPad::GetPadButtonData(0, MYINPUTPAD::XINPUT_PAD::BUTTON_A) == 1)		// １Pが入力された
				{
					controllNumber = 0;
					controllCount = 0;
				}
				if (MYINPUTPAD::InputPad::GetPadButtonData(1, MYINPUTPAD::XINPUT_PAD::BUTTON_A) == 1)		// ２Pが入力された
				{
					controllNumber = 1;
					controllCount = 0;
				}
				if (MYINPUTPAD::InputPad::GetPadButtonData(2, MYINPUTPAD::XINPUT_PAD::BUTTON_A) == 1)		// ３Pが入力された
				{
					controllNumber = 2;
					controllCount = 0;
				}
				if (MYINPUTPAD::InputPad::GetPadButtonData(3, MYINPUTPAD::XINPUT_PAD::BUTTON_A) == 1)		// ４Pが入力された
				{
					controllNumber = 3;
					controllCount = 0;
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
			manager->Update(controllNumber);
		}
	}

	// 削除
	delete manager;

	DxLib::DxLib_End();		// DXライブラリの後始末

	return 0;
}