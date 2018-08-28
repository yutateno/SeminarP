#include "Stage.hpp"

// コンストラクタ
Stage::Stage()
{
	// ステージの読み込み
	LoadFile::MyLoad("media\\ステージモデル\\move1_hantei.fyn", drawStageHandle, ELOADFILE::mv1model);
	LoadFile::MyLoad("media\\ステージモデル\\move1_hantei.fyn", collStageHandle, ELOADFILE::mv1model);

	MV1SetScale(drawStageHandle, VGet(1.75f, 1.0f, 1.75f));

	// 座標を指定
	MV1SetPosition(drawStageHandle, VGet(0, 0, 0));
}

// デストラクタ
Stage::~Stage()
{
	MV1DeleteModel(drawStageHandle);
}


// 要らないロードの削除処理
void Stage::LoadInit()
{
	MV1DeleteModel(collStageHandle);
}

// 描画
void Stage::Draw()
{
	MV1DrawModel(drawStageHandle);

	int i;
	VECTOR Pos1;
	VECTOR Pos2;
	float LINE_AREA_SIZE = 100000.0f;
	int LINE_NUM = 50;

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
}

// 当たり判定モデルを渡す用ゲッター
int Stage::GetCollStageHandle()
{
	return collStageHandle;
}
