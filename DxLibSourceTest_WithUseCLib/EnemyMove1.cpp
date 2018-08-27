#include "EnemyMove1.hpp"

EnemyMove1::EnemyMove1(int collStageHandle) : BasicActor(collStageHandle)
{
	// ３Ｄモデルの読み込み
	LoadFile::MyLoad("media\\光る玉\\sphere_jamp.fyn", modelHandle, ELOADFILE::mv1model);

	// ３Ｄモデルの0番目のアニメーションをアタッチする
	attachNum = 0;
	attachMotion = MV1AttachAnim(modelHandle, attachNum, -1, FALSE);

	// アタッチしたアニメーションの総再生時間を取得する
	totalTime = MV1GetAttachAnimTotalTime(modelHandle, attachMotion);

	// モデルの基本情報
	modelHeight = 80.0f;
	modelWigth = 15.0f;

	// モデルの向きと位置
	area = VGet(400.0f, 0.0f, 0.0f);
	preArea = area;

	// 足元の影に関する
	shadowHeight = 5.0f;
	shadowSize = 15.0f;

	// それぞれの速度
	//walkSpeed = 0.0f;
	animSpeed = 1.0f;

	// モデルの座標を更新
	MV1SetPosition(modelHandle, area);
}

EnemyMove1::~EnemyMove1()
{
	if (modelHandle != -1)
	{
		MV1DeleteModel(modelHandle);
	}
}

void EnemyMove1::Draw()
{
	BasicActor::Draw();

	DrawCapsule3D(area, VAdd(area, VGet(0.0f, modelHeight, 0.0f)), modelWigth, 8, GetColor(0, 255, 0), GetColor(255, 255, 255), false);		// 当たり判定を確認用の表示テスト
}

void EnemyMove1::Process()
{
	// モーションの実態
	Player_AnimProcess();

	// ステージのあたり判定
	StageHit();

	Player_PlayAnim(0);
	
	// 指定位置にモデルを配置
	MV1SetPosition(modelHandle, area);
}