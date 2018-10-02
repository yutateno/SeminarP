#include "Camera.hpp"

using namespace MY_XINPUT;

// �R���X�g���N�^
Camera::Camera(const VECTOR charaarea, const int collStageHandle)
{
	stageHandle = MV1DuplicateModel(collStageHandle);
	MV1SetScale(stageHandle, VGet(1.75f, 1.0f, 1.75f));
	MV1SetupCollInfo(stageHandle, -1);									// ���f���̃R���W���������Z�b�g�A�b�v(-1�ɂ��S�̃t���[��)
	MV1SetPosition(stageHandle, VGet(0.0f, 0.0f, 0.0f));				// �X�e�[�W�̍��W���X�V
	MV1SetFrameVisible(stageHandle, -1, false);							// �X�e�[�W��`�悳���Ȃ��i�ł��ǂ���Draw�Ă΂Ȃ����炱��Ӗ��Ȃ��C������j
	MV1RefreshCollInfo(stageHandle, -1);								// �X�e�[�W��`�悳���Ȃ��i�ł��ǂ���Draw�Ă΂Ȃ����炱��Ӗ��Ȃ��C������j

	cameraArea = VGet(0, 350, 500);
	viewArea = VGet(0, 150, 0);

	charaArea = VAdd(charaarea, VGet(0.0f, 80.0f, 0.0f));

	speed = DX_PI_F / 90;
	angle = 0.0f;

	SetCameraNearFar(100.0f, 10000.0f);	// �J�����̕`��͈͂��w��

	// �������̎��_����������̃^�[�Q�b�g������p�x�ɃJ������ݒu
	SetCameraPositionAndTarget_UpVecY(VAdd(cameraArea, charaArea), VAdd(viewArea, charaArea));
}

// �f�X�g���N�^
Camera::~Camera()
{
	MODEL_RELEASE(stageHandle);
}


// ���C���v���Z�X
void Camera::Process(const VECTOR charaarea, const unsigned __int8 controllNumber)
{
	VECTOR TestPosition = cameraArea;
	//static int zoom = 0;

	charaArea = VAdd(charaarea, VGet(0.0f, 80.0f, 0.0f));					// �L�����̈ʒu���X�V��������

	// ���ɉ�]��
	if (KeyData::Get(KEY_INPUT_LEFT) >= 1
		|| InputPad::GetPadThumbData(controllNumber, STICK_RIGHT_X) < 0)
	{
		RLrotate(speed, TestPosition);	// ��]����
		angle += speed;
	}
	// �E�ɉ�]��
	if (KeyData::Get(KEY_INPUT_RIGHT) >= 1
		|| InputPad::GetPadThumbData(controllNumber, STICK_RIGHT_X) > 0)
	{
		RLrotate(-speed, TestPosition);	// ��]����
		angle -= speed;
	}

	// ��L�[��������Ă����牺���猩�グ��
	if (KeyData::Get(KEY_INPUT_UP) >= 1
		|| InputPad::GetPadThumbData(controllNumber, STICK_RIGHT_Y) > 0)
	{
		// ����
		if (TestPosition.y > 240)
		{
			TestPosition = VAdd(TestPosition, VScale(VNorm(TestPosition), -10));	// �P�ʃx�N�g�������ă}�C�i�X�����ē�������Ɍ��炷
		}
	}

	// ���L�[��������Ă�����ォ�猩���낷
	if (KeyData::Get(KEY_INPUT_DOWN) >= 1
		|| ::InputPad::GetPadThumbData(controllNumber, STICK_RIGHT_Y) < 0)
	{
		// ����
		if (TestPosition.y < 400)
		{
			TestPosition = VAdd(TestPosition, VScale(VNorm(TestPosition), 10));	// VScale����Ȃ�
		}
	}

	//MV1_COLL_RESULT_POLY_DIM HRes;
	//int HitNum;

	//// �����_����J�����̍��W�܂ł̊ԂɃX�e�[�W�̃|���S�������邩���ׂ�
	//HRes = MV1CollCheck_Capsule(this->stageHandle, -1, VAdd(TestPosition, charaArea), charaArea, 10.0f);
	//HitNum = HRes.HitNum;
	//MV1CollResultPolyDimTerminate(HRes);
	//if (HitNum != 0)
	//{
	//	//TestPosition = VAdd(TestPosition, VAdd(VGet(0, 10.0f, 0), VScale(VNorm(VGet(TestPosition.x, 0, TestPosition.z)), -1)));	// �Y�[���C��������������
	//	//zoom++;
	//	cameraArea = TestPosition;

	//	//float NotHitLength;
	//	//float HitLength;
	//	//float TestLength;

	//	//// �������疳���ʒu�܂Ńv���C���[�ɋ߂Â�

	//	//// �|���S���ɓ�����Ȃ��������Z�b�g
	//	//NotHitLength = 0.0f;

	//	//// �|���S���ɓ����鋗�����Z�b�g
	//	//HitLength = sqrt((charaArea.x - cameraArea.x) * (charaArea.x - cameraArea.x) + (charaArea.z - cameraArea.z) * (charaArea.z - cameraArea.z));;
	//	//do
	//	//{
	//	//	// �����邩�ǂ����e�X�g���鋗�����Z�b�g( ������Ȃ������Ɠ����鋗���̒��� )
	//	//	TestLength = NotHitLength + (HitLength - NotHitLength) / 2.0f;
	//	//	
	//	//	// �V�������W�ŕǂɓ����邩�e�X�g
	//	//	HRes = MV1CollCheck_Capsule(this->stageHandle, -1, this->charaArea, TestPosition, 10.0f);
	//	//	HitNum = HRes.HitNum;
	//	//	MV1CollResultPolyDimTerminate(HRes);
	//	//	if (HitNum != 0)
	//	//	{
	//	//		// ���������瓖���鋗���� TestLength �ɕύX����
	//	//		HitLength = TestLength;
	//	//	}
	//	//	else
	//	//	{
	//	//		// ������Ȃ������瓖����Ȃ������� TestLength �ɕύX����
	//	//		NotHitLength = TestLength;
	//	//	}

	//	//	// HitLength �� NoHitLength ���\���ɋ߂Â��Ă��Ȃ������烋�[�v
	//	//} while (HitLength - NotHitLength > 0.1f);
	//}
	//else
	//{
	//	if (zoom > 0)
	//	{
	//		TestPosition = VAdd(TestPosition, VAdd(VGet(0, -10.0f, 0), VScale(VNorm(VGet(TestPosition.x, 0, TestPosition.z)), 1)));	// �Y�[���A�E�g������������
	//		zoom--;
	//	}
	//	cameraArea = TestPosition;
	//}

	cameraArea = TestPosition;

#ifdef _CAMERA_DEBG
	printfDx("%d\n", HitNum);
	//DrawCapsule3D(VAdd(cameraArea, charaArea), VAdd(viewArea, charaArea), 5.0f, 8, GetColor(0, 255, 0), GetColor(255, 255, 255), false);		// �ʔ���
	DrawCapsule3D(VAdd(cameraArea, charaArea), charaArea, 5.0f, 8, GetColor(0, 255, 0), GetColor(255, 255, 255), false);		// �����蔻����m�F�p�̕\���e�X�g
#endif // !_CAMERA_DEBG

}

void Camera::SetUp()
{
	SetupCamera_Perspective(60.0f * DX_PI_F / 180.0f);

	SetCameraNearFar(100.0f, 10000.0f);	// �J�����̕`��͈͂��w��

	// �������̎��_����������̃^�[�Q�b�g������p�x�ɃJ������ݒu
	SetCameraPositionAndTarget_UpVecY(VAdd(cameraArea, charaArea), VAdd(viewArea, charaArea));
}

// ang�p��]����
void Camera::RLrotate(const float speed, VECTOR& p_cameraArea)
{
	float tempX = p_cameraArea.x;
	p_cameraArea.x = tempX * cosf(speed) + p_cameraArea.z *sinf(speed);
	p_cameraArea.z = -tempX * sinf(speed) + p_cameraArea.z * cosf(speed);
}

// �J�����̃A���O����n���Q�b�^�[
const float Camera::GetAngle() const
{
	return angle;
}