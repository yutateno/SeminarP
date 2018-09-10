#include "MainMove1.hpp"


// ���f�����Ƃ̂����蔻�菈��
void MainMove1::ActorHit()
{
	for (int i = 0; i < enemyNum; ++i)
	{
		// ������������ɂȂ�����
		if (BaseMove::GetDistance(p_character->GetArea(), s_enemyAggre[i].p_enemyMove->GetArea()) <= 60
			&& s_enemyAggre[i].aliveNow)
		{
			s_enemyAggre[i].aliveNow = false;			// ���������Ȃ�
			catchEnemyNum++;				// ����������J�E���g
		}
	}
}



// ���C�g�Ɋւ���
void MainMove1::LightProcess()
{
	for (int i = 0; i != lightNum; ++i)
	{
		SetLightRangeAttenHandle(lightHandle[i], lightRange[i], 0.0f, 0.002f, 0.0f);
		SetLightPositionHandle(lightHandle[i], lightArea[i]);
	}

	if (catchEnemyNum == 30)
	{
		lightEventCount = 0;
		lightEventStart = true;
	}
	else if (catchEnemyNum == 29 && !lightEventStart && !lightEventEnd)
	{
		lightEventCount = 0;
		lightRangeSpeed = 20.0f;
		lightRangePreMax = 5100.0f;
		lightEventStart = true;
	}
	else if (catchEnemyNum == 20)
	{
		lightEventEnd = false;
	}
	else if (catchEnemyNum == 19 && !lightEventStart && !lightEventEnd)
	{
		lightEventCount = 0;
		lightRangeSpeed = 14.0f;
		lightRangePreMax = 3700.0f;
		lightEventStart = true;
	}
	else if (catchEnemyNum == 13)
	{
		lightEventEnd = false;
	}
	else if (catchEnemyNum == 12 && !lightEventStart && !lightEventEnd)
	{
		lightEventCount = 0;
		lightRangeSpeed = 11.0f;
		lightRangePreMax = 2600.0f;
		lightEventStart = true;
	}
	else if (catchEnemyNum == 8)
	{
		lightEventEnd = false;
	}
	else if (catchEnemyNum == 7 && !lightEventStart && !lightEventEnd)
	{
		lightEventCount = 0;
		lightRangeSpeed = 9.0f;
		lightRangePreMax = 1700.0f;
		lightEventStart = true;
	}
	else if (catchEnemyNum == 5)
	{
		lightEventEnd = false;
	}
	else if (catchEnemyNum == 4 && !lightEventStart && !lightEventEnd)
	{
		lightRange[0] = 1000.0f;
		lightRange[1] = 1000.0f;
		lightRange[2] = 1000.0f;
		lightRange[3] = 1000.0f;
		lightEventCount = 0;
		lightRangeSpeed = 7.0f;
		lightRangePreMax = 1000.0f;
		lightEventStart = true;
	}
	else if (catchEnemyNum == 3)
	{
		SetLightEnableHandle(lightHandle[2], TRUE);
	}
	else if (catchEnemyNum == 2)
	{
		SetLightEnableHandle(lightHandle[1], TRUE);
	}
	else if(catchEnemyNum == 1)
	{
		SetLightEnableHandle(lightHandle[0], TRUE);
	}

	if (lightEventStart)
	{
		lightEventCount++;
		lightRange[0] = lightRangePreMax + lightEventCount * lightRangeSpeed;
		lightRange[1] = lightRangePreMax + lightEventCount * lightRangeSpeed;
		lightRange[2] = lightRangePreMax + lightEventCount * lightRangeSpeed;
		lightRange[3] = lightRangePreMax + lightEventCount * lightRangeSpeed; 
		

		if (lightEventCount >= 100)
		{
			lightEventStart = false;
			lightEventEnd = true;
		}


		if (catchEnemyNum == 4)
		{
			if (lightEventCount >= 10)
			{
				SetLightEnableHandle(lightHandle[3], TRUE);
			}
		}
		else if (catchEnemyNum == 12)
		{
			backgroundColor = GetColor(lightEventCount, lightEventCount, lightEventCount);
		}
		else if (catchEnemyNum == 19)
		{
			lightArea[0].z = -630.0f + lightEventCount * 5;
			lightArea[1].z = -630.0f + lightEventCount * 4;
			lightArea[2].z = -630.0f + lightEventCount * 3;
			lightArea[3].z = -630.0f + lightEventCount * 2;
			backgroundColor = GetColor(lightEventCount / 2 + 100, lightEventCount / 2 + 100, lightEventCount / 2 + 100);
		}
		else if (catchEnemyNum == 29)
		{
			backgroundColor = GetColor(lightEventCount / 2 + 150, lightEventCount / 2 + 150, lightEventCount / 2 + 150);
		}
		else if (catchEnemyNum == 30)
		{
			lightEventCount++;
			backgroundColor = GetColor(lightEventCount + 150, lightEventCount + 150, lightEventCount + 150);
			// �t�F�[�h�C���t�F�[�h�A�E�g�֘A
			SetDrawBlendMode(DX_BLENDMODE_ALPHA, 255 - 255);
			DrawGraph(0, 0, drawWhite, false);
			SetDrawBlendMode(DX_BLENDMODE_NOBLEND, 255);
			if (lightEventCount >= 100)
			{
				lightEventStart = false;
				lightEventEnd = true;
			}
		}
	}
}


// �R���X�g���N�^
MainMove1::MainMove1(std::vector<int> v_file)
{
	SetLightEnable(FALSE);		// ���R��������؎Ւf


	// �t�H�O�Ɋւ���
	SetFogEnable(TRUE);					// �t�H�O��L���ɂ���
	SetFogColor(128, 128, 128);			// �t�H�O�̐F�ɂ���
	SetFogStartEnd(8000.0f, 10000.0f);	// �t�H�O�̊J�n����


	// �w�i�F�Ɋւ���
	backgroundColor = GetColor(0, 0, 0);


	// �|�C���^NULL������
	p_camera	 = NULL;
	p_character	 = NULL;
	p_stage		 = NULL;
	for (int i = 0; i != enemyNum; ++i)
	{
		s_enemyAggre[i].p_enemyMove = NULL;
	}


	// �G�ȊO�̃|�C���^�̏�����
	p_stage		 = new Stage(v_file[EFILE1::drawStage]);									// �X�e�[�W������
	p_character	 = new Character(v_file[EFILE1::character], v_file[EFILE1::collStage]);		// �L�����N�^�[������
	p_camera	 = new Camera(p_character->GetArea(), v_file[EFILE1::collStage]);			// �J����������


	// �G�����Ɋւ���
	std::random_device rnd;     // �񌈒�I�ȗ���������𐶐�
	std::mt19937 mt(rnd());     // �����Z���k�E�c�C�X�^��32�r�b�g��
	std::uniform_int_distribution<> randInX(-4000, 4000);        // X���W�p����
	std::uniform_int_distribution<> randInZ(-4000, 4000);        // Z���W�p����
	std::uniform_int_distribution<> color(1, 100);				 // �F�p�̗���
	for (int i = 0; i != enemyNum; ++i)
	{
		s_enemyAggre[i].aliveNow = true;
		s_enemyAggre[i].p_enemyMove = new EnemyMove1(v_file[1], (float)randInX(mt), (float)randInZ(mt), color(mt) / 100.0f);		// �G������
	}
	catchEnemyNum = 0;		// �G����ɓ��ꂽ����������
;

	// ���C�g�Ɋւ���
	for (int i = 0; i != lightNum; ++i)
	{
		lightHandle[i] = 0;
		lightArea[i] = p_character->GetArea();						// ���C�g�̈ʒu������
		lightRange[i] = 1700.0f;
		lightArea[0].y = -200.0f;
		lightArea[i].z = -630.0f;															// ���C�g�̑傫��������
		lightHandle[i] = CreatePointLightHandle(lightArea[i], lightRange[i], 0.0f, 0.002f, 0.0f);	// ���C�g�𐶐�
		SetLightEnableHandle(lightHandle[i], FALSE);
	}
	lightEventStart = false;
	lightEventEnd = false;
	lightEventCount = 0;
	lightRangePreMax = 0.0f;
	lightRangeSpeed = 0.0f;


	// �t�F�[�h�C���p�z���C�g�摜
	drawWhite = 0;
	drawWhite = v_file[EFILE1::feedWhite];
}


// �f�X�g���N�^
MainMove1::~MainMove1()
{
	if (drawWhite != 0)
	{
		DeleteGraph(drawWhite);
	}
	for (int i = 0; i != lightNum; ++i)
	{
		if (lightHandle[i] != -1 || lightHandle[i] != 0)
		{
			DeleteLightHandle(lightHandle[i]);
			lightHandle[i] = 0;
		}
	}
	for (int i = 0; i < enemyNum; ++i)
	{
		if (s_enemyAggre[i].p_enemyMove != NULL)
		{
			delete s_enemyAggre[i].p_enemyMove;
			s_enemyAggre[i].p_enemyMove = NULL;
		}
	}
	if (p_camera != NULL)
	{
		delete p_camera;
		p_camera = NULL;
	}
	if (p_character != NULL)
	{
		delete p_character;
		p_character = NULL;
	}
	if (p_stage != NULL)
	{
		delete p_stage;
		p_stage = NULL;
	}
}


// �`��
void MainMove1::Draw()
{
	DrawBox(0, 0, 1920, 1080, backgroundColor, true);	// �w�i��`��
	

	p_stage->Draw();					// �X�e�[�W��`��


	p_character->Draw();				// �L�����N�^�[��`��


	for (int i = 0; i < enemyNum; ++i)
	{
		if (s_enemyAggre[i].aliveNow)
		{
			s_enemyAggre[i].p_enemyMove->Draw();		// �G��`��
		}
	}


#ifdef _DEBUG
#ifdef _SEARCH_MODEL_DEBUG
	for (int i = 0; i < enemyNum; ++i)
	{
		//DrawFormatString(0, i * 16, GetColor(255, 255, 255), "%d", BaseMove::GetDistance(character->GetArea(), enemyAggre[i].enemyMove->GetArea()));
		if (BaseMove::GetDistance(p_character->GetArea(), s_enemyAggre[i].p_enemyMove->GetArea()) <= 500)
		{
			DrawLine3D(VAdd(p_character->GetArea(), VGet(0.0f, 80.0f, 0.0f)), VAdd(s_enemyAggre[i].p_enemyMove->GetArea(), VGet(0.0f, 60.0f, 0.0f)), GetColor(255, 0, 0));
		}
	}
#endif
	printfDx("%d\n", catchEnemyNum);
#endif // _DEBUG

}


// ���C���v���Z�X
void MainMove1::Process(unsigned __int8 controllNumber)
{
	p_character->Process(controllNumber, p_camera->GetAngle());		// �L�����N�^�[�̃v���Z�X���Ă�


	p_camera->Process(p_character->GetArea(), controllNumber);		// �J�����̃v���Z�X���Ă�


	for (int i = 0; i < enemyNum; ++i)
	{
		// �����Ă�����
		if (s_enemyAggre[i].aliveNow)
		{
			s_enemyAggre[i].p_enemyMove->Process();					// �G�̃v���Z�X���Ă�
		}
	}


	ActorHit();		// �A�N�^�[���Ƃ̂����蔻����Ă�
	

	LightProcess();		// ���C�g�̃v���Z�X���Ă�


#ifdef _DEBUG
	DebugKeyControll();
#endif // _DEBUG
}


#ifdef _DEBUG
void MainMove1::DebugKeyControll()
{
	if (KeyData::Get(KEY_INPUT_Z) == 1)
	{
		if (catchEnemyNum != 30)
		{
			catchEnemyNum++;
		}
		s_enemyAggre[catchEnemyNum - 1].aliveNow = false;
	}
	if (KeyData::Get(KEY_INPUT_H) == 1)
	{
		SetLightEnable(TRUE);
	}
	if (KeyData::Get(KEY_INPUT_I) >= 1)
	{
		for (int i = 0; i != lightNum; ++i)
		{
			lightRange[i] += 10.0f;
			SetLightRangeAttenHandle(lightHandle[i], lightRange[i], 0.0f, 0.002f, 0.0f);
		}
	}
	if (KeyData::Get(KEY_INPUT_K) >= 1)
	{
		for (int i = 0; i != lightNum; ++i)
		{
			lightRange[i] -= 10.0f;
			SetLightRangeAttenHandle(lightHandle[i], lightRange[i], 0.0f, 0.002f, 0.0f);
		}
	}
	if (KeyData::Get(KEY_INPUT_R) >= 1)
	{
		for (int i = 0; i != lightNum; ++i)
		{
			lightArea[i].y += 10.0f;
			SetLightRangeAttenHandle(lightHandle[i], lightRange[i], 0.0f, 0.002f, 0.0f);
		}
	}
	if (KeyData::Get(KEY_INPUT_F) >= 1)
	{
		for (int i = 0; i != lightNum; ++i)
		{
			lightArea[i].y -= 10.0f;
			SetLightRangeAttenHandle(lightHandle[i], lightRange[i], 0.0f, 0.002f, 0.0f);
		}
	}
	if (KeyData::Get(KEY_INPUT_W) >= 1)
	{
		for (int i = 0; i != lightNum; ++i)
		{
			lightArea[i].z += 10.0f;
			SetLightPositionHandle(lightHandle[i], lightArea[i]);
		}
	}
	if (KeyData::Get(KEY_INPUT_S) >= 1)
	{
		for (int i = 0; i != lightNum; ++i)
		{
			lightArea[i].z -= 10.0f;
			SetLightPositionHandle(lightHandle[i], lightArea[i]);
		}
	}
	if (KeyData::Get(KEY_INPUT_A) >= 1)
	{
		for (int i = 0; i != lightNum; ++i)
		{
			lightArea[i].x -= 10.0f;
			SetLightPositionHandle(lightHandle[i], lightArea[i]);
		}
	}
	if (KeyData::Get(KEY_INPUT_D) >= 1)
	{
		for (int i = 0; i != lightNum; ++i)
		{
			lightArea[i].x += 10.0f;
			SetLightPositionHandle(lightHandle[i], lightArea[i]);
		}
	}
}
#endif // _DEBUG
