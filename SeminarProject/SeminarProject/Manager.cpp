#include "Manager.hpp"


void Manager::SceneChange()
{
	switch (nowScene)
	{
	case ESceneNumber::FIRSTMOVE:
		baseMove = new MainMove1(loadThread->GetFile());
		baseMove->SetScene(nowScene);
		delete loadThread;
		break;
	case ESceneNumber::STARTLOAD:
		loadThread = new LoadThread();
		break;
	default:
		break;
	}
}

Manager::Manager()
{
	preScene = ESceneNumber::STARTLOAD;
	nowScene = ESceneNumber::STARTLOAD;

	move1str[0] = "media\\�X�e�[�W���f��\\move1_graphic.myn";
	move1str[1] = "media\\�X�e�[�W���f��\\move1_hantei.myn";
	move1str[2] = "media\\CLPH\\motion\\CLPH_motionALL.myn";
	load1[0] = ELOADFILE::mv1model;
	load1[1] = ELOADFILE::mv1model;
	load1[2] = ELOADFILE::mv1model;

	baseMove = NULL;
	loadThread = new LoadThread();
}


Manager::~Manager()
{
	if (baseMove != NULL)
	{
		delete baseMove;
	}
}

void Manager::Update(unsigned __int8 controllNumber)
{
	if (nowScene == preScene)		// ���̃V�[���ƒ��O�̃V�[��������
	{
		if (preScene == ESceneNumber::STARTLOAD)		// ���[�h���ɕς�����u�ԂȂ�
		{
			loadThread->Run(max1, move1str, load1);		// ���[�h������
			if (loadThread->num >= max1)		// ���[�h���I��������
			{
				if (KeyData::Get(KEY_INPUT_Z) == 1)			// �I�������ꑀ��
				{
					nowScene = ESceneNumber::FIRSTMOVE;
				}
			}
		}
		else
		{
			baseMove->Draw();
			baseMove->Process(controllNumber);
			nowScene = baseMove->GetScene();
		}
	}
	else
	{
		SceneChange();
		preScene = nowScene;
	}
}
