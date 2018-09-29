#include "DropItemMove1.hpp"


DropItemMove1::DropItemMove1(const int draw, const int collStageHandle) : BasicObject(collStageHandle)
{
	area = VGet(1000.0f, 0.0f, 1000.0f);

	BasicObject::shadowHeight = 10.0f;
	BasicObject::shadowSize = 5.0f;


	this->modelHandle = 0;
	this->modelHandle = MV1DuplicateModel(draw);

	MV1SetScale(this->modelHandle, VGet(0.2f, 0.2f, 0.2f));

	// モデルの座標を更新
	MV1SetPosition(this->modelHandle, area);
}


DropItemMove1::~DropItemMove1()
{
	MODEL_RELEASE(modelHandle);
}

void DropItemMove1::Draw()
{
	BasicObject::ShadowFoot();


	BasicObject::Draw();

#ifdef _DROPITEM1_DEBUG
	DrawCapsule3D(area, VAdd(area, VGet(0.0f, 1.0f, 0.0f)), 80.0f, 8, GetColor(0, 255, 0), GetColor(255, 255, 255), false);		// 当たり判定を確認用の表示テスト
#endif // _DROPITEM1_DEBUG
}
