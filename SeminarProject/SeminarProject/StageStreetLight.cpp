#include "StageStreetLight.hpp"


StageStreetLight::StageStreetLight(const int draw, const int collStageHandle, const VECTOR area) : BasicObject(collStageHandle)
{
	this->area = area;


	shadowHeight = 10.0f;
	shadowSize = 5.0f;

	this->modelHandle = 0;
	this->modelHandle = MV1DuplicateModel(draw);


	MV1SetPosition(this->modelHandle, area);
}

StageStreetLight::~StageStreetLight()
{
	MODEL_RELEASE(modelHandle);
}

void StageStreetLight::Draw()
{
	BasicObject::ShadowFoot();


	BasicObject::Draw();
}
