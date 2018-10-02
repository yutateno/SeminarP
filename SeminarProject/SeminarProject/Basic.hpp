#pragma once

// �C���N���[�h�t�@�C��
#include "InputKey.hpp"
#include <math.h>



// DEBUG�}�N��

// ���f���̃T�[�N���f�o�b�O�\��
//#define _MODEL_DEBUG

// ���߃��f�����T�[�`�������ʂ��o���T�[�N���f�o�b�O�\��
//#define _SEARCH_MODEL_DEBUG

// �G���A�̃f�o�b�O�p��
//#define _AREA_DEBUG

// �L�����N�^�[�N���X�̃f�o�b�O
//#define _CHARACTER_DEBUG

// ���[�u�P�N���X�̃f�o�b�O
//#define _MOVE1_DEBUG

// ���[�u�P�ŗ����Ă錕�̃N���X�̃f�o�b�O
//#define _DROPITEM1_DEBUG

// �J�����̃f�o�b�O
#define _CAMERA_DEBG



// �}�N��

// directx�֌W�ł悭�������}�N��
#define SAFE_RELEASE(p) { if(p!=NULL) { (p)->Release(); (p)=NULL; } }

// �|�C���^�p����}�N��
#define POINTER_RELEASE(p) { if(p!=NULL) {delete (p); (p)=NULL; } }

// 2D�֘A�摜����}�N��
#define GRAPHIC_RELEASE(p) { if(p!=-1) { DeleteGraph(p); (p)=-1; } }

// 3D�֘A�摜����}�N��
#define MODEL_RELEASE(p) { if(p!=-1) { MV1DeleteModel(p); (p)=-1; } }

// ������}�N��
#define SOUND_RELEASE(p) { if(p!=-1) { StopSoundMem(p); DeleteSoundMem(p); (p)=-1; } }

// �����n���h������}�N��
#define LIGHT_RELEASE(p) { if(p!=-1) { DeleteLightHandle(p); (p)=-1; } }

// �V���h�E�}�b�v����}�N��
#define SHADOW_MAP_RELEASE(p) { if(p!=-1) { DeleteShadowMap(p); (p)=-1; } }
