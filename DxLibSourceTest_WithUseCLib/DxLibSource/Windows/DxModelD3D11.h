// -------------------------------------------------------------------------------
// 
// 		�c�w���C�u����		���f���f�[�^����v���O����( Direct3D11 )�w�b�_�t�@�C��
// 
// 				Ver 3.18f
// 
// -------------------------------------------------------------------------------

#ifndef __DXMODELD3D11_H__
#define __DXMODELD3D11_H__

#include "../DxCompileConfig.h"

#ifndef DX_NON_GRAPHICS

#ifndef DX_NON_DIRECT3D11

// �C���N���[�h ------------------------------------------------------------------

#ifndef DX_NON_MODEL

#include "../DxLib.h"
#include "../DxModel.h"
#include "DxDirectX.h"

#ifndef DX_NON_NAMESPACE

namespace DxLib
{

#endif // DX_NON_NAMESPACE

// �}�N����` --------------------------------------------------------------------

// �\���̒�` --------------------------------------------------------------------

// Direct3D11�p ���f���f�[�^�Ǘ��p�\����
struct MV1_MODEL_MANAGE_DIRECT3D11
{
	void *					CommonBuffer ;						// ��ɃV�F�C�v���W�̍X�V�Ŏg�p����ėp�o�b�t�@
	int						CommonBufferSize ;					// ��ɃV�F�C�v���W�̍X�V�Ŏg�p����ėp�o�b�t�@�̃T�C�Y
} ;

// Direct3D11�p���_�o�b�t�@���
struct MV1_VERTEXBUFFER_DIRECT3D11
{
	D_ID3D11Buffer *		VertexBuffer ;						// ���_�o�b�t�@
	D_ID3D11Buffer *		IndexBuffer ;						// �C���f�b�N�X�o�b�t�@
} ;

// Direct3D11�p�g���C�A���O�����X�g��f�[�^���
struct MV1_TRIANGLE_LIST_BASE_DIRECT3D11
{
	BYTE					SkinFreeBoneVertexBufferUpdate ;	// �X�{�[���ȏ�̃X�L�j���O���b�V�������p���_�f�[�^���X�V�������ǂ���( TRUE:�X�V�ς�  FALSE:���X�V )
	void *					SkinFreeBoneVertexBuffer ;			// �X�{�[���ȏ�̃X�L�j���O���b�V�������p���_�f�[�^
} ;

// Direct3D11�p�g���C�A���O�����X�g���
struct MV1_TRIANGLE_LIST_DIRECT3D11
{
	void *					SkinFreeBoneVertexPositionBuffer ;	// �X�{�[���ȏ�̃X�L�j���O���b�V�������p���_���W�f�[�^
	D_ID3D11Buffer *		VertexBuffer ;						// ���_�o�b�t�@
} ;

// �������ϐ��錾 --------------------------------------------------------------

extern MV1_MODEL_MANAGE_DIRECT3D11 MV1Man_D3D11 ;

// �֐��v���g�^�C�v�錾-----------------------------------------------------------

// �ėp�o�b�t�@�֐�
extern	int				MV1_D3D11_CommonBuffer_Setup( int Size ) ;						// �w��T�C�Y�̔ėp�o�b�t�@�̏������s��
extern	int				MV1_D3D11_CommonBuffer_Terminate( void ) ;						// �ėp�o�b�t�@�̌�n�����s��

// ���ˑ��֐�
extern	int				MV1_D3D11_Terminate_PF( void ) ;																// ���f���@�\�̌�n��
extern	int				MV1_D3D11_TerminateModelBaseHandle_PF( MV1_MODEL_BASE *ModelBase ) ;							// ���f���f�[�^�n���h���̌�n��
extern	int				MV1_D3D11_TerminateTriangleListBaseTempBuffer_PF( MV1_TRIANGLE_LIST_BASE *MBTList ) ;				// �g���C�A���O�����X�g�̈ꎞ�����p�̃o�b�t�@���J������
extern	void			MV1_D3D11_SetupPackDrawInfo_PF( MV1_MODEL_BASE *ModelBase ) ;									// ���������`��֌W�̏����Z�b�g�A�b�v����
extern	int				MV1_D3D11_SetupVertexBufferBase_PF( int MV1ModelBaseHandle, int DuplicateNum = 1, int ASyncThread = FALSE ) ;	// ���f����f�[�^�̒��_�o�b�t�@�̃Z�b�g�A�b�v������( -1:�G���[ )
extern	int				MV1_D3D11_SetupVertexBuffer_PF( int MHandle, int ASyncThread = FALSE ) ;						// ���f���f�[�^�̒��_�o�b�t�@�̃Z�b�g�A�b�v������( -1:�G���[ )
extern	int				MV1_D3D11_TerminateVertexBufferBase_PF( int MV1ModelBaseHandle ) ;								// ���_�o�b�t�@�̌�n��������( -1:�G���[ )
extern	int				MV1_D3D11_TerminateVertexBuffer_PF( int MV1ModelHandle ) ;										// ���_�o�b�t�@�̌�n��������( -1:�G���[ )
extern	int				MV1_D3D11_SetupShapeVertex_PF( int MHandle ) ;													// �V�F�C�v�f�[�^�̃Z�b�g�A�b�v������
extern	int				MV1_D3D11_BeginRender_PF( MV1_MODEL *Model ) ;													// �R�c���f���̃����_�����O�̏������s��
extern	int				MV1_D3D11_EndRender_PF( void ) ;																// �R�c���f���̃����_�����O�̌�n�����s��
extern	void			MV1_D3D11_DrawMesh_PF( MV1_MESH *Mesh, int TriangleListIndex = -1 ) ;							// ���b�V���`�敔���𔲂��o��������

#ifndef DX_NON_NAMESPACE

}

#endif // DX_NON_NAMESPACE

#endif // DX_NON_MODEL

#endif // DX_NON_DIRECT3D11

#endif // DX_NON_GRAPHICS

#endif // __DXMODELD3D11_H__
