#include "DxLib.h"
#include "InputPad.hpp"
#include "InputKey.hpp"

int WINAPI WinMain(HINSTANCE hInstance, HINSTANCE hPrevInstance, LPSTR lpCmdLine, int nCmdShow)
{
#ifdef _DEBUG
	SetOutApplicationLogValidFlag(TRUE);	// ログテキスト出力する
#elif NDEBUG
	SetOutApplicationLogValidFlag(FALSE);	// ログテキスト出力しない
#endif


	SetWindowText("SeminarProject");	// メインウインドウのウインドウタイトルを変更する
	
	ChangeWindowMode(TRUE);			// ウィンドウズモードにさせるかどうか

	SetGraphMode(640, 480, 32);					// 画像に合わせて画面サイズを変更

	if (DxLib_Init() == -1)		// ＤＸライブラリ初期化処理
	{
		return -1;			// エラーが起きたら直ちに終了
	}

	SetDrawScreen(DX_SCREEN_BACK);	// 背景描画

	MYINPUTPAD::Input::Input();
	MYINPUTPAD::Input::Update();
	KeyData::UpDate();
	int model = MV1LoadModel("media\\姫_変身体\\モーション\\attack.fbx");

	// 画面に映る位置に３Ｄモデルを移動
	MV1SetPosition(model, VGet(320.0f, 50.0f, 30.0f));

	// ３Ｄモデルの０番目のアニメーションをアタッチする
	int AttachIndex = MV1AttachAnim(model, 0, -1, FALSE);

	// アタッチしたアニメーションの総再生時間を取得する
	float TotalTime = MV1GetAttachAnimTotalTime(model, AttachIndex);

	// 再生時間の初期化
	float PlayTime = 0.0f;

	while (ScreenFlip() == 0 && ProcessMessage() == 0 && ClearDrawScreen() == 0 && KeyData::CheckEnd() != 0)
	{
		KeyData::UpDate();
		MYINPUTPAD::Input::Update();

		// 再生時間を進める
		PlayTime += 0.5f;

		// 再生時間がアニメーションの総再生時間に達したら再生時間を０に戻す
		if (PlayTime >= TotalTime)
		{
			PlayTime = 0.0f;
		}

		// 再生時間をセットする
		MV1SetAttachAnimTime(model, AttachIndex, PlayTime);

		// ３Ｄモデルの描画
		MV1DrawModel(model);
	}

	// モデルハンドルの削除
	MV1DeleteModel(model);

	DxLib::DxLib_End();

	return 0;
}