#include "EnemyMove1.hpp"

// 動きのプロセス
void EnemyMove1::MoveProcess()
{
	// 上昇させる
	if (upNow)
	{
		// 上昇中
		if (60.0f > area.y)
		{
			area.y += flyMove;
		}
		// 上昇しきったら
		else
		{
			upNow = false;
			flyMove = -0.1f;
		}
	}
	// 下降させる
	else
	{
		// 下降中
		if (area.y > 30.0f)
		{
			area.y -= flyMove;
		}
		// 下降しきったら
		else
		{
			upNow = true;
			flyMove = -0.1f;
		}
	}
	// 始発をゆっくりさせる
	if (flyMove < 0.5f)
	{
		flyMove += 0.05f;
	}
}


// コンストラクタ
EnemyMove1::EnemyMove1(int collStageHandle, float areaX, float areaZ) : BasicActor(collStageHandle)
{
	// ３Ｄモデルの読み込み
	LoadFile::MyLoad("media\\光る玉\\sphere.fyn", modelHandle, ELOADFILE::fbxmodel);
	
	// モデルの基本情報
	modelHeight = 10.0f;
	modelWigth = 15.0f;

	// モデルの向きと位置
	this->area = VGet(areaX, 40.0f, areaZ);

	// 足元の影に関する
	shadowHeight = 15.0f;
	shadowSize = 15.0f;

	upNow = true;
	flyMove = 0.0f;
	
	// モデルの座標を更新
	MV1SetPosition(modelHandle, area);
}

// デストラクタ
EnemyMove1::~EnemyMove1()
{
	if (modelHandle != -1)
	{
		MV1DeleteModel(modelHandle);
	}
}


// 描画
void EnemyMove1::Draw()
{
	BasicActor::Draw();		// 基本的なものを引っ張ってくる

#ifdef _MODEL_DEBUG
	VECTOR viewArea = VAdd(area, VGet(0.0f, 60.0f, 0.0f));		// モデルの初期Y座標が浮いているので調整

	DrawCapsule3D(viewArea, VAdd(viewArea, VGet(0.0f, modelHeight, 0.0f)), modelWigth, 8, GetColor(0, 255, 0), GetColor(255, 255, 255), false);		// 当たり判定を確認用の表示テスト
#endif // _MODEL_DEBUG
}

// メインプロセス
void EnemyMove1::Process()
{
	MV1SetMaterialDifColor(modelHandle, 0, GetColorF(1.0f, 1.0f, 1.0f, 1.0f));
	MV1SetMaterialEmiColor(modelHandle, 0, GetColorF(1.0f, 1.0f, 1.0f, 0.0f));

	// 動きのプロセス
	MoveProcess();

	// 指定位置にモデルを配置
	MV1SetPosition(modelHandle, area);
}