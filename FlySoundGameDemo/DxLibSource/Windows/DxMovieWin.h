// -------------------------------------------------------------------------------
// 
// 		�c�w���C�u����		WindowsOS�p����v���O�����w�b�_�t�@�C��
// 
// 				Ver 3.18f
// 
// -------------------------------------------------------------------------------

#ifndef __DXMOVIEWIN_H__
#define __DXMOVIEWIN_H__

#include "../DxCompileConfig.h"

#ifndef DX_NON_MOVIE

// �C���N���[�h ------------------------------------------------------------------
#include "DxUseCStrmBaseFilter.h"
#include "DxGuid.h"
#include "../DxLib.h"
#include "../DxFile.h"

#ifndef DX_NON_NAMESPACE

namespace DxLib
{

#endif // DX_NON_NAMESPACE

// �}�N����` --------------------------------------------------------------------

// �\���̒�` --------------------------------------------------------------------

// ���[�r�[�O���t�B�b�N���ˑ��f�[�^�^
struct MOVIEGRAPH_PF
{
	int						Dummy ;

#ifndef DX_NON_MEDIA_FOUNDATION
	D_IMFSourceReader		*pMFReader ;					// �\�[�X���[�_�[
	D_IMFMediaType			*pMFMediaTypeVideoStream ;		// �ϊ��O�̃r�f�I�X�g���[�����f�B�A����
	D_IMFMediaType			*pMFMediaTypeOutputVideoStream ;// �ϊ���̃r�f�I�X�g���[�����f�B�A����
	UINT32					MFFrameSizeX ;					// ��
	UINT32					MFFrameSizeY ;					// ����
	UINT32					MFFrameRateNumerator ;			// �t���[�����[�g( ���q )
	UINT32					MFFrameRateDenominator ;		// �t���[�����[�g( ���� )
	UINT32					MFAspectRatioX ;				// �A�X�y�N�gX
	UINT32					MFAspectRatioY ;				// �A�X�y�N�gY
	LONGLONG				MFLastReadSampleTimeStamp ;		// �Ō�ɍs���� ReadSample �̃^�C���X�^���v
	int						MFLastReadSampleFrame ;			// �Ō�ɍs���� ReadSample �̃t���[��
	D_PROPVARIANT			MFDuration ;					// ����̒���
	int						MFTotalFrame ;					// ���t���[����
	void *					MFImageBuffer ;					// �C���[�W�ۑ��p�o�b�t�@

	int						MFCurrentFrame ;				// �\�����Ă���t���[��
	LONGLONG				MFPrevTimeCount ;				// �O��̌v������
	LONGLONG				MFPlayNowTime ;					// �Đ�����
	int						MFLoopType ;					// ���[�v�^�C�v( 0:����f�[�^�ɍ��킹�ă��[�v  1:�T�E���h�f�[�^�ɍ��킹�ă��[�v )
	int						MFSetupGraphHandleImage ;		// �摜�\�z�̌�A�O���t�B�b�N�n���h���̃Z�b�g�A�b�v���I���Ă��邩�ǂ���( TRUE:�I���Ă���  FALSE:�I���Ă��Ȃ� )
#ifndef DX_NON_SOUND
	int						MFSoundHandle ;					// �T�E���h�n���h��
	int						MFSoundFrequency ;				// �T�E���h�̎��g��
	int						MFSoundTotalTime ;				// �T�E���h�f�[�^�̍Đ�������( �~���b )
#endif // DX_NON_SOUND
	double					MFPlaySpeedRate ;				// �Đ����x

	void					*MFYBuffer ;					// �x�C���[�W�ւ̃A�h���X
	void					*MFUVBuffer ;					// �t�u�C���[�W�ւ̃A�h���X
	UINT32					MFYWidth, MFYHeight ;			// �x�C���[�W�̕��ƍ���
	UINT32					MFYStride ;						// �x�o�b�t�@�̃s�b�`
	UINT32					MFUVWidth, MFUVHeight ;			// �t�u�C���[�W�̕��ƍ���
	UINT32					MFUVStride ;					// �t�u�o�b�t�@�̃s�b�`

//	BASEIMAGE				BaseImage ;						// �J�����g�t���[�����i�[���ꂽ�t���[���X�^�b�N���̃C���[�W�̃R�s�[
	volatile int			MFBaseImageSetup ;				// �J�����g�t���[���� RGB �C���[�W���\�z����Ă��邩�ǂ���( 1:����Ă���  0:����Ă��Ȃ� )
#ifndef DX_NON_FILTER
	volatile int			MFNotUseYUVGrHandle ;			// YUV�J���[�̃O���t�B�b�N�n���h�����g�p���Ȃ����ǂ���
	volatile int			MFYUVGrHandleSetup ;			// YUV�J���[�̃O���t�B�b�N�n���h���̃Z�b�g�A�b�v���������Ă��邩�ǂ���( 1:�������Ă���  0:�������Ă��Ȃ� )
#endif // DX_NON_FILTER
#endif // DX_NON_MEDIA_FOUNDATION

#ifndef DX_NON_DSHOW_MOVIE
	D_IGraphBuilder			*pGraph ;						// �t�B���^�O���t�}�l�[�W��
	D_IMediaControl			*pMediaControl ;				// ���f�B�A�R���g���[��
	D_IMediaSeeking			*pMediaSeeking ;				// ���f�B�A�V�[�L���O
	D_IBasicAudio			*pBasicAudio ;					// BasicAudio �C���^�[�t�F�C�X

//	D_ISampleGrabber		*SampleGrabber ;				// �T���v���O���b�o�I�u�W�F�N�g

	D_CMovieRender			*pMovieImage ;					// ����̃C���[�W
#endif // DX_NON_DSHOW_MOVIE

#if !defined( DX_NON_MEDIA_FOUNDATION ) || !defined( DX_NON_DSHOW_MOVIE )
	D_STREAM_TIME			FrameTime ;						// �P�t���[��������̎���
	int						UseTemporaryFile ;				// �e���|�����t�@�C�����g�p���Ă��邩�ǂ����A�t���O
	wchar_t					FileName[ FILEPATH_MAX ] ;		// �t�@�C���l�[��
#endif // !defined( DX_NON_MEDIA_FOUNDATION ) || !defined( DX_NON_DSHOW_MOVIE )
} ;

// ���[�r�[�f�[�^���ˑ��Ǘ��\����
struct MOVIEGRAPHMANAGE_PF
{
	int						Dummy ;

#ifndef DX_NON_MEDIA_FOUNDATION
	int						MFStartupRunFlag ;				// MFStartup ���Ă�ł��邩( TRUE:�Ă�  FALSE:�܂��Ă�ł��Ȃ� )
#endif // DX_NON_MEDIA_FOUNDATION
} ;

// �������ϐ��錾 --------------------------------------------------------------

// �֐��v���g�^�C�v�錾-----------------------------------------------------------

#ifndef DX_NON_NAMESPACE

}

#endif // DX_NON_NAMESPACE

#endif // DX_NON_MOVIE

#endif // __DXMOVIEWIN_H__
