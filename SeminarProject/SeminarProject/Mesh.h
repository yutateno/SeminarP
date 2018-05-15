#pragma once

#include <stdio.h>
#include <Windows.h>
#include <D3D11.h>
#include <D3DX10.h>
#include <D3DX11.h>
#include <D3Dcompiler.h>

#pragma comment(lib, "winmm.lib")
#pragma comment(lib, "d3dx10.lib")
#pragma comment(lib, "d3d11.lib")
#pragma comment(lib, "d3dx11.lib")
#pragma comment(lib, "d3dCompiler.lib")

#define SAFE_RELEASE(x) if(x){x->Release(); x=0;}
#define SAFE_DELETE(x) if(x){delete x; x=0;}
#define SAFE_DELETE_ARRAY(x) if(x){delete[] x; x=0;}
#define MAX_LENGTH 100

// ���_�̍\����
struct MY_VERTEX
{
	D3DXVECTOR3 vPos;
	D3DXVECTOR3 vNorm;
	D3DXVECTOR2 vTex;
};

// Simple�V�F�[�_�[�̃A�v�����\����
struct SIMPLECONSTANT_BUFFER0
{
	D3DXMATRIX mW;	// ���[���h���W
	D3DXMATRIX mWVP;	// ���[���h�r���[�|�W�V�����̕ϊ����W
	D3DXVECTOR4 vLightPos[MAX_LENGTH];	// ���C�g�ʒu
	D3DXVECTOR4 vEye;	// �J�������W
};
struct SIMPLECONSTANT_BUFFER1
{
	D3DXVECTOR4 vAmbient;	// �A���r�G���g��
	D3DXVECTOR4 vDiffuse;	// �f�B�t���[�Y�F
	D3DXVECTOR4 vSpecular;	// �X�y�L�����\
};

// �}�e���A���\����
struct MY_MATERIAL
{
	CHAR szName[110];
	D3DXVECTOR4 Ka;	// �A���r�G���g��
	D3DXVECTOR4 Kd;	// �f�B�t���[�Y�F
	D3DXVECTOR4	Ks;	// �X�y�L�����\
	CHAR szTextureName[110];	// �e�N�X�`���\�̃t�@�C����
	ID3D11ShaderResourceView* pTexture;
	DWORD dwNumFace;	// �}�e���A���̃|���S����
	MY_MATERIAL()
	{
		ZeroMemory(this, sizeof(MY_MATERIAL));	// ������
	}
	~MY_MATERIAL()
	{
		SAFE_RELEASE(pTexture);		// ���
	}
};

class Mesh
{
public:
	Mesh();
	~Mesh();
};

