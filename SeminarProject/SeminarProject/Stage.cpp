#include "Stage.hpp"

// コンストラクタ
Stage::Stage(const int drawStageHandle)
{
	this->drawStageHandle = MV1DuplicateModel(drawStageHandle);

	MV1SetScale(this->drawStageHandle, VGet(1.75f, 1.0f, 1.75f));

	// 座標を指定
	MV1SetPosition(this->drawStageHandle, VGet(0, 0, 0));
}

// デストラクタ
Stage::~Stage()
{
	MODEL_RELEASE(drawStageHandle);
}


// 描画
void Stage::Draw()
{
	MV1DrawModel(drawStageHandle);

#ifdef _AREA_DEBUG
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
#endif // _AREA_DEBUG
}
