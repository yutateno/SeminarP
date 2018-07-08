#include "DxLib.h"

int WINAPI WinMain(HINSTANCE hInstance, HINSTANCE hPrevInstance, LPSTR lpCmdLine, int nCmdShow)
{
	int ModelHandle, AttachIndex;
	float TotalTime, PlayTime;
	int Direct3DVersion;

	ChangeWindowMode(true);

	// ＤＸライブラリの初期化
	if (DxLib_Init() < 0)
	{
		// エラーが発生したら直ちに終了
		return -1;
	}

	// ３Ｄモデルの読み込み
	ModelHandle = MV1LoadModel("media\\姫_変身体\\モーション\\TestYvee.fbx");

	// 描画先を裏画面に変更
	SetDrawScreen(DX_SCREEN_BACK);
	
	// 使用している Direct3D のバージョンを取得
	Direct3DVersion = GetUseDirect3DVersion();

	// 画面に映る位置に３Ｄモデルを移動
	MV1SetPosition(ModelHandle, VGet(320.0f, 70.0f, -70.0f));

	// ３Ｄモデルの０番目のアニメーションをアタッチする
	AttachIndex = MV1AttachAnim(ModelHandle, 0, -1, FALSE);

	// アタッチしたアニメーションの総再生時間を取得する
	TotalTime = MV1GetAttachAnimTotalTime(ModelHandle, AttachIndex);

	// 再生時間の初期化
	PlayTime = 0.0f;

	// 何かキーが押されるかウインドウが閉じられるまでループ
	while (ProcessMessage() == 0 && CheckHitKeyAll() == 0)
	{
		// 画面をクリア
		ClearDrawScreen();

		// 再生時間を進める
		PlayTime += 0.5f;

		// 再生時間がアニメーションの総再生時間に達したら再生時間を０に戻す
		if (PlayTime >= TotalTime)
		{
			PlayTime = 0.0f;
		}

		// 再生時間をセットする
		MV1SetAttachAnimTime(ModelHandle, AttachIndex, PlayTime);

		// ３Ｄモデルの描画
		MV1DrawModel(ModelHandle);

		DrawFormatString(0, 100, 255, "%d", ModelHandle);

		// 使用バージョンが Direct3D 9Ex かどうかをチェック
		if (Direct3DVersion == DX_DIRECT3D_9EX)
		{
			DrawString(0, 0, "Direct3D 9Ex を使用しています", GetColor(255, 255, 255));
		}
		else if (Direct3DVersion == DX_DIRECT3D_11)
		{
			DrawString(0, 0, "Direct3D 11 を使用しています", GetColor(255, 255, 255));
		}
		else
		{
			DrawString(0, 0, "Direct3D  を使用して", GetColor(255, 255, 255));
		}

		// 裏画面の内容を表画面に反映
		ScreenFlip();
	}

	// モデルハンドルの削除
	MV1DeleteModel(ModelHandle);

	// ＤＸライブラリの後始末
	DxLib_End();

	// ソフトの終了
	return 0;
}