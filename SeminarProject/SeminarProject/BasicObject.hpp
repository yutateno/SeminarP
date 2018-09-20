#pragma once
#include "LoadFile.hpp"


class BasicObject
{
protected:
	float shadowHeight;			// �����e�̍���
	float shadowSize;			// �����e�̃T�C�Y
	void ShadowFoot();			// �����e�̃v���Z�X
	VECTOR area;				// �����e�̃G���A


	int modelHandle;	// ���f���̃n���h��


	float modelHeight;	// ���f���̍���


private:
	MV1_COLL_RESULT_POLY_DIM ShadowHitResDim;		// ���͂̃|���S����������\����
	MV1_COLL_RESULT_POLY *ShadowHitRes;				// �e�̃|���S���̍\����
	VERTEX3D ShadowVertex[3];
	VECTOR ShadowSlideVec;
	int shadowHandle;


	int stageHandle;


public:
	BasicObject(const int collStageHandle);
	virtual ~BasicObject();


	const VECTOR GetArea() const;

	void Draw();		// �`��
};

