#include "StageStreetLight.hpp"


StageStreetLight::StageStreetLight(int draw, int collStageHandle, VECTOR area) : BasicObject(collStageHandle)
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
	if (modelHandle != -1)
	{
		MV1DeleteModel(modelHandle);
		modelHandle = 0;
	}
}

void StageStreetLight::Draw()
{
	BasicObject::ShadowFoot();


	BasicObject::Draw();
}
