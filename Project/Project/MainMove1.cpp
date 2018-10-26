#include "MainMove1.hpp"


// ���f�����Ƃ̂����蔻�菈��
void MainMove1::ActorHit()
{
	for (int i = 0; i < enemyNum; ++i)
	{
		// ������������ɂȂ�����
		if (s_enemyAggre[i].aliveNow)
		{
			if (BaseMove::GetDistance(p_character->GetArea(), s_enemyAggre[i].p_enemyMove->GetArea()) <= 60)
			{
				s_enemyAggre[i].aliveNow = false;			// ���������Ȃ�
				catchEnemyNum++;				// ����������J�E���g
			}
			else if (BaseMove::GetDistance(p_character->GetArea(), s_enemyAggre[i].p_enemyMove->GetArea()) <= 300)
			{
				s_enemyAggre[i].p_enemyMove->StolenChara(p_character->GetArea());
			}
		}
	}

	if (catchEnemyNum == 30 && lightEventEnd
		&& BaseMove::GetDistance(p_character->GetArea(), p_dropItem->GetArea()) <= 60)
	{
		touchSword = true;
	}
}



// ���C�g�Ɋւ���
void MainMove1::LightProcess()
{
	// �����̏����X�V������
	for (int i = 0; i != lightNum; ++i)
	{
		SetLightRangeAttenHandle(lightHandle[i], lightRange[i], 0.0f, 0.002f, 0.0f);		// �����̍L�����X�V
		SetLightPositionHandle(lightHandle[i], lightArea[i]);								// �����̈ʒu���X�V
	}


	// �ʂ̌��Ŕ��f������
	switch (catchEnemyNum)
	{
	case 1:		// ��ڂ�L���ɂ���
		SetLightEnableHandle(lightHandle[0], TRUE);
		break;


	case 2:		// ��ڂ�L���ɂ���
		SetLightEnableHandle(lightHandle[1], TRUE);
		break;


	case 3:		// �O�ڂ�L���ɂ���
		SetLightEnableHandle(lightHandle[2], TRUE);
		break;


	case 4:		// �l�ڂ�L���ɂ��n�ʂ̖��邳�����킶��L����
		if (!lightEventStart && !lightEventEnd)
		{
			lightEventCount = 0;
			lightRangeSpeed = 7.0f;
			lightRangePreMax = 1000.0f;
			lightEventStart = true;
		}
		break;


	case 5:		// �t���O��|��
		lightEventEnd = false;
		break;


	case 7:		// �n�ʂ̖��邳���L������
		if (!lightEventStart && !lightEventEnd)
		{
			lightEventCount = 0;
			lightRangeSpeed = 9.0f;
			lightRangePreMax = 1700.0f;
			lightEventStart = true;
		}
		break;


	case 8:		// �t���O��|��
		lightEventEnd = false;
		break;


	case 12:	// �n�ʂ̖��邳���L������
		if (!lightEventStart && !lightEventEnd)
		{
			lightEventCount = 0;
			lightRangeSpeed = 11.0f;
			lightRangePreMax = 2600.0f;
			lightEventStart = true;
		}
		break;


	case 13:	// �t���O��|��
		lightEventEnd = false;
		break;


	case 19:	// �n�ʂ̖��邳���L������
		if (!lightEventStart && !lightEventEnd)
		{
			lightEventCount = 0;
			lightRangeSpeed = 14.0f;
			lightRangePreMax = 3700.0f;
			lightEventStart = true;
		}
		break;


	case 20:	// �t���O��|��
		lightEventEnd = false;
		break;


	case 29:	// �n�ʂ̖��邳���L������
		if (!lightEventStart && !lightEventEnd)
		{
			lightEventCount = 0;
			lightRangeSpeed = 20.0f;
			lightRangePreMax = 5100.0f;
			lightEventStart = true;
		}
		break;


	case 30:	// �����������Ď��R������ʏ�ɂ���
		if (!lightEnd && !lightEventStart)
		{
			lightEventCount = 0;
			lightEventStart = true;
			lightEnd = true;
		}
		break;


	default:
		break;
	}


	// �C�x���g���J�n����
	if (lightEventStart)
	{
		lightEventCount++;		// �J�E���g�𑱂���

		// �����̖��邳���L����
		lightRange[0] = lightRangePreMax + lightEventCount * lightRangeSpeed;
		lightRange[1] = lightRangePreMax + lightEventCount * lightRangeSpeed;
		lightRange[2] = lightRangePreMax + lightEventCount * lightRangeSpeed;
		lightRange[3] = lightRangePreMax + lightEventCount * lightRangeSpeed;


		// �J�E���g���K��ȏ�ɂȂ�����
		if (lightEventCount >= 100)
		{
			lightEventStart = false;		// �C�x���g�̊J�n�̃t���O��|��
			lightEventEnd = true;			// �C�x���g�̏I���̃t���O���グ��


			// �S�Ă̋ʂ���ɓ��ꂽ��
			if (catchEnemyNum == 30)
			{
				for (int i = 0; i != lightNum; ++i)
				{
					DeleteLightHandle(lightHandle[i]);		// ����������
					lightHandle[i] = 0;
					SetLightEnable(TRUE);					// ���R������L���ɂ���
					p_character->PositionReset();			// �L�����N�^�[�̃|�W�V���������ɖ߂�
				}
			}
		}


		// ���ɂ��C�x���g�̈Ⴂ
		switch (catchEnemyNum)
		{
		case 4:			// �n�ʂ𖾂邭����
			if (lightEventCount >= 10)
			{
				SetLightEnableHandle(lightHandle[3], TRUE);		// �l�ڂ̌������o��
			}
			break;


		case 12:		// �w�i�𖾂邭����
			backgroundColor = GetColor(lightEventCount, lightEventCount, lightEventCount);
			break;


		case 19:		// �w�i�𖾂邭���������J�����̕����ɓ�����
			lightArea[0].z = -630.0f + lightEventCount * 5;
			lightArea[1].z = -630.0f + lightEventCount * 4;
			lightArea[2].z = -630.0f + lightEventCount * 3;
			lightArea[3].z = -630.0f + lightEventCount * 2;
			backgroundColor = GetColor(lightEventCount / 2 + 100, lightEventCount / 2 + 100, lightEventCount / 2 + 100);
			break;


		case 29:		// �w�i�𖾂邭����
			backgroundColor = GetColor(lightEventCount / 2 + 150, lightEventCount / 2 + 150, lightEventCount / 2 + 150);
			break;


		case 30:		// �w�i�𖾂邭���܂Ԃ��������Ă�ԂɌ����������Ď��R�����ɐ؂�ւ���
			backgroundColor = GetColor(lightEventCount + 155, lightEventCount + 155, lightEventCount + 155);
			// �t�F�[�h�A�E�g�̏����������Ă܂Ԃ�������
			if (lightEventCount < 50)
			{
				SetDrawBlendMode(DX_BLENDMODE_ALPHA, 255 - (lightEventCount * 2));
				//DrawGraph(0, 0, drawWhite, false);
				DrawBox(0, 0, 1920, 1080, GetColor(255, 255, 255), true);
				SetDrawBlendMode(DX_BLENDMODE_NOBLEND, 255);
			}
			else		// �t�F�[�h�C���̏����������Ė߂�
			{
				SetDrawBlendMode(DX_BLENDMODE_ALPHA, 155 + ((lightEventCount - 50) * 2));
				//DrawGraph(0, 0, drawWhite, false);
				DrawBox(0, 0, 1920, 1080, GetColor(255, 255, 255), true);
				SetDrawBlendMode(DX_BLENDMODE_NOBLEND, 255);
			}
			break;

		}
	}
}


// �R���X�g���N�^
MainMove1::MainMove1(const std::vector<int> v_file)
{
	SetLightEnable(FALSE);		// ���R��������؎Ւf


	// �t�H�O�Ɋւ���
	SetFogEnable(FALSE);					// �t�H�O��L���ɂ���
	//SetFogColor(128, 128, 128);			// �t�H�O�̐F�ɂ���
	//SetFogStartEnd(8000.0f, 10000.0f);	// �t�H�O�̊J�n����


	// �w�i�F�Ɋւ���
	backgroundColor = GetColor(0, 0, 0);


	// �|�C���^NULL������
	p_camera = NULL;
	p_character = NULL;
	p_stage = NULL;
	p_dropItem = NULL;
	for (int i = 0; i != enemyNum; ++i)
	{
		s_enemyAggre[i].p_enemyMove = NULL;
	}


	// �G�ȊO�̃|�C���^�̏�����
	p_stage = new Stage(v_file[EFILE::drawStage]);									// �X�e�[�W������
	p_character = new Character(v_file[EFILE::character], v_file[EFILE::collStage]);		// �L�����N�^�[������
	p_camera = new Camera(p_character->GetArea(), v_file[EFILE::collStage]);			// �J����������
	p_dropItem = new DropItemMove1(v_file[EFILE::sword], v_file[EFILE::collStage]);


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
	lightEnd = false;


	touchSword = false;


	soundBG = v_file[EFILE::sound];
	ChangeVolumeSoundMem(150, soundBG);
	PlaySoundMem(soundBG, DX_PLAYTYPE_LOOP);
}


// �f�X�g���N�^
MainMove1::~MainMove1()
{
	SOUND_RELEASE(soundBG);
	for (int i = 0; i != lightNum; ++i)
	{
		LIGHT_RELEASE(lightHandle[i]);
	}
	for (int i = 0; i < enemyNum; ++i)
	{
		POINTER_RELEASE(s_enemyAggre[i].p_enemyMove);
	}
	POINTER_RELEASE(p_dropItem);
	POINTER_RELEASE(p_camera);
	POINTER_RELEASE(p_character);
	POINTER_RELEASE(p_stage);
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


	if (catchEnemyNum == 30)
	{
		p_dropItem->Draw();				// ����`��
	}

#ifdef _MOVE1_DEBUG
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
	//printfDx("NUM:%d\tCOUNT:%d\tX:%f\tY:%f\tZ:%f\n", catchEnemyNum, lightEventCount, p_character->GetArea().x, p_character->GetArea().y, p_character->GetArea().z);
#endif // _MOVE1_DEBUG

}


// ���C���v���Z�X
void MainMove1::Process(const unsigned __int8 controllNumber)
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


void MainMove1::CameraProcess()
{
	p_camera->SetUp();
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

	if (KeyData::Get(KEY_INPUT_K) == 1)
	{
		scene = ESceneNumber::SECONDLOAD;
	}
}
#endif // _DEBUG
