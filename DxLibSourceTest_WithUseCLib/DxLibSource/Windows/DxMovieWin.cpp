//-----------------------------------------------------------------------------
// 
// 		�c�w���C�u����		WindowsOS�p����v���O����
// 
//  	Ver 3.18f
// 
//-----------------------------------------------------------------------------

// �c�w���C�u�����쐬���p��`
#define __DX_MAKE

#include "../DxCompileConfig.h"

#ifndef DX_NON_MOVIE

// �C���N���[�h----------------------------------------------------------------
#include "DxMovieWin.h"
#include "DxWinAPI.h"
#include "DxFileWin.h"
#include "../DxGraphics.h"
#include "../DxLog.h"
#include "../DxMovie.h"
#include "../DxSound.h"
#include "../DxSystem.h"

#ifndef DX_NON_NAMESPACE

namespace DxLib
{

#endif // DX_NON_NAMESPACE

// �}�N����`------------------------------------------------------------------

// �^��`----------------------------------------------------------------------

// �f�[�^�錾------------------------------------------------------------------

#ifndef DX_NON_MEDIA_FOUNDATION
static GUID *g_MFVideoFormatGUIDTable[ D_MFVIDEOFORMAT_TYPE_UNKNOWN ] =
{
	&D_MFVIDEOFORMAT_BASE,
	&D_MFVIDEOFORMAT_RGB32,
	&D_MFVIDEOFORMAT_ARGB32,
	&D_MFVIDEOFORMAT_RGB24,
	&D_MFVIDEOFORMAT_RGB555,
	&D_MFVIDEOFORMAT_RGB565,
	&D_MFVIDEOFORMAT_RGB8,
	&D_MFVIDEOFORMAT_AI44,
	&D_MFVIDEOFORMAT_AYUV,
	&D_MFVIDEOFORMAT_YUY2,
	&D_MFVIDEOFORMAT_YVYU,
	&D_MFVIDEOFORMAT_YVU9,
	&D_MFVIDEOFORMAT_UYVY,
	&D_MFVIDEOFORMAT_NV11,
	&D_MFVIDEOFORMAT_NV12,
	&D_MFVIDEOFORMAT_YV12,
	&D_MFVIDEOFORMAT_I420,
	&D_MFVIDEOFORMAT_IYUV,
	&D_MFVIDEOFORMAT_Y210,
	&D_MFVIDEOFORMAT_Y216,
	&D_MFVIDEOFORMAT_Y410,
	&D_MFVIDEOFORMAT_Y416,
	&D_MFVIDEOFORMAT_Y41P,
	&D_MFVIDEOFORMAT_Y41T,
	&D_MFVIDEOFORMAT_Y42T,
	&D_MFVIDEOFORMAT_P210,
	&D_MFVIDEOFORMAT_P216,
	&D_MFVIDEOFORMAT_P010,
	&D_MFVIDEOFORMAT_P016,
	&D_MFVIDEOFORMAT_v210,
	&D_MFVIDEOFORMAT_v216,
	&D_MFVIDEOFORMAT_v410,
	&D_MFVIDEOFORMAT_MP43,
	&D_MFVIDEOFORMAT_MP4S,
	&D_MFVIDEOFORMAT_M4S2,
	&D_MFVIDEOFORMAT_MP4V,
	&D_MFVIDEOFORMAT_WMV1,
	&D_MFVIDEOFORMAT_WMV2,
	&D_MFVIDEOFORMAT_WMV3,
	&D_MFVIDEOFORMAT_WVC1,
	&D_MFVIDEOFORMAT_MSS1,
	&D_MFVIDEOFORMAT_MSS2,
	&D_MFVIDEOFORMAT_MPG1,
	&D_MFVIDEOFORMAT_DVSL,
	&D_MFVIDEOFORMAT_DVSD,
	&D_MFVIDEOFORMAT_DVHD,
	&D_MFVIDEOFORMAT_DV25,
	&D_MFVIDEOFORMAT_DV50,
	&D_MFVIDEOFORMAT_DVH1,
	&D_MFVIDEOFORMAT_DVC,
	&D_MFVIDEOFORMAT_H264,
	&D_MFVIDEOFORMAT_MJPG,
} ;

#endif // DX_NON_MEDIA_FOUNDATION

// �֐��v���g�^�C�v�錾 -------------------------------------------------------

#ifndef DX_NON_MEDIA_FOUNDATION
// Media Foundation ���g�p��������t�@�C���̃I�[�v�������݂�
static int MediaFoundationOpenMovieFile(
	MOVIEGRAPH * Movie,
	const wchar_t *FileName,
	int *Width,
	int *Height,
	int ASyncThread
) ;

// �J�����g�t���[����RGB�摜���쐬����( �߂�l  1:�쐬���ꂽ  0:����Ȃ����� )
static int MediaFoundationMovie_SetupImage(
	MOVIEGRAPH * Movie,
	int BaseImage,
	int YUVGrHandle,
	int ASyncThread
) ;

// Media Foundation ���g�p��������t�@�C���̃t���[����i�߂�
static int MediaFoundationMovie_IncToFrame(
	MOVIEGRAPH * Movie,
	int AddFrame,
	int ASyncThread
) ;
#endif

#ifndef DX_NON_DSHOW_MOVIE
// DirectShow ���g�p��������t�@�C���̃I�[�v�������݂�
extern int DirectShowOpenMovieFile(
	MOVIEGRAPH * Movie,
	const wchar_t *FileName,
	int *Width,
	int *Height,
	int SurfaceMode,
	int ASyncThread
) ;
#endif

// �v���O����------------------------------------------------------------------

// ���[�r�[�֘A�̊Ǘ������̏������̊��ˑ�����
extern int InitializeMovieManage_PF( void )
{
	return 0 ;
}

// ���[�r�[�֘A�̊Ǘ������̌�n���̊��ˑ�����
extern int TerminateMovieManage_PF( void )
{
#ifndef DX_NON_MEDIA_FOUNDATION
	// MFStartup ���Ă΂�Ă����� MFShutdown ���Ă�
	if( MovieGraphManageData.PF.MFStartupRunFlag )
	{
		MovieGraphManageData.PF.MFStartupRunFlag = FALSE ;

		if( WinAPIData.Win32Func.MFShutdownFunc != NULL )
		{
			WinAPIData.Win32Func.MFShutdownFunc() ;
		}
	}

#endif // DX_NON_MEDIA_FOUNDATION

	return 0 ;
}

#ifndef DX_NON_MEDIA_FOUNDATION

// Media Foundation ���g�p��������t�@�C���̃I�[�v�������݂�
static int MediaFoundationOpenMovieFile(
	MOVIEGRAPH * Movie,
	const wchar_t *FileName,
	int *Width,
	int *Height,
	int ASyncThread
)
{
	HRESULT hr ;
	wchar_t ErStr[ 256 ] ;

	Movie->NowImage.GraphData = NULL;
	Movie->YUVFlag = FALSE ;
	Movie->OverlayDestX = 0 ;
	Movie->OverlayDestY = 0 ;
	Movie->OverlayDestExRate = 0 ;
	Movie->OverlayDispFlag = 0 ;
	Movie->FirstUpdateFlag = FALSE ;
	_MEMSET( &Movie->OverlaySrcRect, 0, sizeof( RECT ) ) ;
	_MEMSET( &Movie->OverlayDestRect, 0, sizeof( RECT ) ) ;

	Movie->PF.MFImageBuffer = NULL ;
	Movie->PF.MFYBuffer = NULL ;
	Movie->PF.MFUVBuffer = NULL ;

	// �Ō�� ReadSample �̎��ԁA�t���[�������Z�b�g
	Movie->PF.MFLastReadSampleTimeStamp = -1 ;
	Movie->PF.MFLastReadSampleFrame = -1 ;

	// Media Foundation �֘A�� DLL ������������G���[
	if( WinAPIData.Win32Func.MFPLATDLL == NULL ||
		WinAPIData.Win32Func.MFREADWRITEDLL == NULL ||
		WinAPIData.Win32Func.MFStartupFunc == NULL ||
		WinAPIData.Win32Func.MFCreateSourceReaderFromURLFunc == NULL ||
		WinAPIData.Win32Func.MFCreateMediaTypeFunc == NULL /*||
		WinAPIData.Win32Func.PropVariantToInt64Func == NULL*/ )
	{
		return -1 ;
	}

	// �܂� MFStartup ���Ă�ł��Ȃ�������Ă�
	if( MovieGraphManageData.PF.MFStartupRunFlag == FALSE )
	{
		MovieGraphManageData.PF.MFStartupRunFlag = TRUE ;

		hr = WinAPIData.Win32Func.MFStartupFunc( D_MF_VERSION, D_MFSTARTUP_FULL ) ;
		if( hr != S_OK )
		{
			_WCSCPY_S( ErStr, sizeof( ErStr ), L"MFStartup Error\n" ) ;
			goto ERR ;
		}
	}

   	// �t�@�C�����ۑ�
	_WCSCPY_S( Movie->PF.FileName, sizeof( Movie->PF.FileName ), FileName ) ;

	// �f�B���N�g���̋�؂肪 / �ɂȂ��Ă���ӏ��� \ �ɒu��������
	{
		wchar_t *wp ;

		for( wp = Movie->PF.FileName; *wp; wp++ )
		{
			if( *wp == L'/' )
			{
				*wp = '\\' ;
			}
		}
	}

	// �t�@�C�����J��
	if( WinAPIData.Win32Func.MFCreateSourceReaderFromURLFunc( Movie->PF.FileName, NULL, &Movie->PF.pMFReader ) != S_OK )
	{
		DWORD_PTR fp ;
		HANDLE FileHandle ;
		void *TempBuffer ;
		size_t CompSize, MoveSize ;
		DWORD WriteSize ;
		size_t FileSize ;
		const DWORD BufferSize = 0x100000 ;

		// ���Ƀe���|�����t�@�C�����쐬���Ă���ꍇ�́A
		// �X�Ƀe���|�����t�@�C�����쐬���邱�Ƃ͂��Ȃ�
		if( Movie->PF.UseTemporaryFile == TRUE )
			goto ERR ;

		// �t�@�C�����J���Ȃ�������A�[�J�C�u����Ă���\��������
#ifdef UNICODE
		fp = DX_FOPEN( Movie->PF.FileName ) ;
#else
		fp = DX_FOPEN( FileName ) ;
#endif
		if( fp == 0 ) goto ERR ;
		
		// �J�����ꍇ�̓e���|�����t�@�C���ɏ����o��
		{
			// �t�@�C���T�C�Y�̎擾
			DX_FSEEK( fp, 0L, SEEK_END ) ;
			FileSize = ( size_t )DX_FTELL( fp ) ;
			DX_FSEEK( fp, 0L, SEEK_SET ) ;

			// �ꎞ�I�Ƀf�[�^���i�[����o�b�t�@���m��
			TempBuffer = DXALLOC( BufferSize ) ;
			if( TempBuffer == NULL )
			{
				DX_FCLOSE( fp ) ;
				goto ERR ;
			}

			// �e���|�����t�@�C���̍쐬
			FileHandle = CreateTemporaryFile( Movie->PF.FileName, sizeof( Movie->PF.FileName ) ) ;

			if( FileHandle == NULL )
			{
				DX_FCLOSE( fp ) ;
				DXFREE( TempBuffer ) ;
				goto ERR ;
			}
			Movie->PF.UseTemporaryFile = TRUE ;

			// �e���|�����t�@�C���Ƀf�[�^�������o��
			CompSize = 0 ;
			while( CompSize < FileSize )
			{
				MoveSize = CompSize - FileSize ;
				if( MoveSize > BufferSize ) MoveSize = BufferSize ;

				DX_FREAD( TempBuffer, MoveSize, 1, fp ) ;
				WriteFile( FileHandle, TempBuffer, ( DWORD )MoveSize, &WriteSize, NULL ) ;

				if( MoveSize != WriteSize ) break ;
				CompSize += MoveSize ;
			}

			// �t�@�C������A���������������
			DX_FCLOSE( fp ) ;
			CloseHandle( FileHandle ) ;
			DXFREE( TempBuffer ) ;
		}

		// ���߂ăt�@�C�����J��
		if( WinAPIData.Win32Func.MFCreateSourceReaderFromURLFunc( Movie->PF.FileName, NULL, &Movie->PF.pMFReader ) != S_OK )
		{
			// �e���|�����t�@�C�����폜
			DeleteFileW( Movie->PF.FileName ) ;
			goto ERR ;
		}
	}

	// �t�@�C���̏����擾����
	if( Movie->PF.pMFReader->GetCurrentMediaType(
			( DWORD )D_MF_SOURCE_READER_FIRST_VIDEO_STREAM, 
			&Movie->PF.pMFMediaTypeVideoStream ) != S_OK )
		goto ERR ;

	// �T�C�Y�̎擾
	if( D_MFGetAttributeSize( Movie->PF.pMFMediaTypeVideoStream, D_MF_MT_FRAME_SIZE, &Movie->PF.MFFrameSizeX, &Movie->PF.MFFrameSizeY ) != S_OK )
		goto ERR ;

	// �t���[�����[�g�̎擾
	if( D_MFGetAttributeRatio( Movie->PF.pMFMediaTypeVideoStream, D_MF_MT_FRAME_RATE, &Movie->PF.MFFrameRateNumerator, &Movie->PF.MFFrameRateDenominator ) != S_OK )
		goto ERR ;

	// �A�X�y�N�g��̎擾
	if( D_MFGetAttributeRatio( Movie->PF.pMFMediaTypeVideoStream, D_MF_MT_PIXEL_ASPECT_RATIO, &Movie->PF.MFAspectRatioX, &Movie->PF.MFAspectRatioY ) != S_OK )
		goto ERR ;

	// �Đ����Ԃ��擾
	if( Movie->PF.pMFReader->GetPresentationAttribute( D_MF_SOURCE_READER_MEDIASOURCE, D_MF_PD_DURATION, &Movie->PF.MFDuration ) != S_OK )
		goto ERR ;
	Movie->StopTime = Movie->PF.MFDuration.hVal.QuadPart ;

	// ���t���[�������Z�o
	{
		LONGLONG Temp64_1, Temp64_2 ;
		DWORD Temp128[ 4 ] ;

		Temp64_1 = Movie->PF.MFFrameRateNumerator ;
		_MUL128_1( ( DWORD * )&Movie->PF.MFDuration.hVal.QuadPart, ( DWORD * )&Temp64_1, Temp128 ) ;
		Temp64_1 = ( LONGLONG )10000000 * ( LONGLONG )Movie->PF.MFFrameRateDenominator ;
		_DIV128_1( Temp128, ( DWORD * )&Temp64_1, ( DWORD * )&Temp64_2 ) ;
		Movie->PF.MFTotalFrame = ( int )Temp64_2 ;
	}

	// �o�͌`�����쐬
	WinAPIData.Win32Func.MFCreateMediaTypeFunc( &Movie->PF.pMFMediaTypeOutputVideoStream ) ;
	Movie->PF.pMFMediaTypeOutputVideoStream->SetGUID( D_MF_MT_MAJOR_TYPE, D_MFMEDIATYPE_VIDEO ) ;
	Movie->PF.pMFMediaTypeOutputVideoStream->SetGUID( D_MF_MT_SUBTYPE, D_MFVIDEOFORMAT_IYUV ) ;
	Movie->PF.pMFMediaTypeOutputVideoStream->SetUINT32( D_MF_MT_DEFAULT_STRIDE, Movie->PF.MFFrameSizeX ) ;
	D_MFSetAttributeRatio( ( D_IMFAttributes * )Movie->PF.pMFMediaTypeOutputVideoStream, D_MF_MT_FRAME_RATE, Movie->PF.MFFrameRateNumerator, Movie->PF.MFFrameRateDenominator ) ;
	D_MFSetAttributeSize( ( D_IMFAttributes * )Movie->PF.pMFMediaTypeOutputVideoStream, D_MF_MT_FRAME_SIZE, Movie->PF.MFFrameSizeX, Movie->PF.MFFrameSizeY ) ;
//	Movie->PF.pMFMediaTypeOutputVideoStream->SetUINT32( D_MF_MT_INTERLACE_MODE, D_MFVideoInterlace_Progressive ) ;
	Movie->PF.pMFMediaTypeOutputVideoStream->SetUINT32( D_MF_MT_ALL_SAMPLES_INDEPENDENT, TRUE ) ;
	D_MFSetAttributeRatio( ( D_IMFAttributes * )Movie->PF.pMFMediaTypeOutputVideoStream, D_MF_MT_PIXEL_ASPECT_RATIO, Movie->PF.MFAspectRatioX, Movie->PF.MFAspectRatioY ) ;

	// �o�͌`�����Z�b�g
	hr = Movie->PF.pMFReader->SetCurrentMediaType(
			( DWORD )D_MF_SOURCE_READER_FIRST_VIDEO_STREAM,
			NULL,
			Movie->PF.pMFMediaTypeOutputVideoStream ) ;
	if( hr != S_OK )
		goto ERR ;

	// �P�t���[���ӂ�̎��Ԃ��Z�b�g
	if( Movie->PF.MFFrameRateNumerator == 0 )
	{
		Movie->PF.FrameTime = 10000000 / 60 ;
	}
	else
	{
		LONGLONG OneSec = ( D_STREAM_TIME )10000000 ;
		Movie->PF.FrameTime = OneSec * ( D_STREAM_TIME )Movie->PF.MFFrameRateDenominator / Movie->PF.MFFrameRateNumerator ;
	}

	// �Đ����x�̏�����
	Movie->PF.MFPlaySpeedRate = 1.0 ;

	// �T�[�t�F�X���[�h�̓m�[�}��
	Movie->SurfaceMode = DX_MOVIESURFACE_NORMAL ;

	// �摜�\�z�̌�A�O���t�B�b�N�n���h���̃Z�b�g�A�b�v���I���Ă��邩�̃t���O��|���Ă���
	Movie->PF.MFSetupGraphHandleImage = FALSE ;

	// �摜�C���[�W�̏����Z�b�g����
	Movie->NowImage.Width        = ( int )Movie->PF.MFFrameSizeX ;
	Movie->NowImage.Height       = ( int )Movie->PF.MFFrameSizeY ;
	Movie->NowImage.Pitch        = ( int )( ( Movie->PF.MFFrameSizeX * 4 + 15 ) / 16 * 16 ) ;
	Movie->PF.MFImageBuffer      = DXALLOC( Movie->NowImage.Pitch * Movie->NowImage.Height ) ;
	if( Movie->PF.MFImageBuffer == NULL )
		goto ERR ;
	_MEMSET( Movie->PF.MFImageBuffer, 0, Movie->NowImage.Pitch * Movie->NowImage.Height ) ;
	Movie->NowImage.GraphData    = Movie->PF.MFImageBuffer ;
	Movie->NowImageGraphOutAlloc = TRUE ;
	NS_CreateXRGB8ColorData( &Movie->NowImage.ColorData ) ;
	Movie->UseNowImage = &Movie->NowImage ;

	// Y�o�b�t�@��UV�o�b�t�@�̊m��
	Movie->PF.MFYWidth  = Movie->PF.MFFrameSizeX ;
	Movie->PF.MFYHeight = Movie->PF.MFFrameSizeY ;
	Movie->PF.MFYStride = ( Movie->PF.MFFrameSizeX + 15 ) / 16 * 16 ;
	Movie->PF.MFYBuffer = DXALLOC( Movie->PF.MFYStride * Movie->PF.MFYHeight ) ;
	if( Movie->PF.MFYBuffer == NULL )
		goto ERR ;
	Movie->PF.MFUVWidth  = Movie->PF.MFFrameSizeX / 2 ;
	Movie->PF.MFUVHeight = Movie->PF.MFFrameSizeY / 2 ;
	Movie->PF.MFUVStride = ( Movie->PF.MFFrameSizeX + 15 ) / 16 * 16 ;
	Movie->PF.MFUVBuffer = DXALLOC( Movie->PF.MFUVStride * Movie->PF.MFUVHeight ) ;
	if( Movie->PF.MFUVBuffer == NULL )
		goto ERR ;

	if( Width  ) *Width  = ( int )Movie->PF.MFFrameSizeX ;
	if( Height ) *Height = ( int )Movie->PF.MFFrameSizeY ;

#ifndef DX_NON_SOUND
	LOADSOUND_GPARAM GParam ;

	InitLoadSoundGParam( &GParam ) ;

	// �J������ Thera �p�̃Z�b�g�A�b�v���s��

	// �T�E���h�Đ��p�ɃT�E���h�f�[�^�Ƃ��Ă��ǂݍ���
	GParam.NotInitSoundMemDelete = TRUE ;
#ifndef DX_NON_OGGVORBIS
	GParam.OggVorbisFromTheoraFile = TRUE ;
#endif // DX_NON_OGGVORBIS
	GParam.DisableReadSoundFunctionMask = ~DX_READSOUNDFUNCTION_MF ;
//	if( FileName != NULL )
//	{
		GParam.CreateSoundDataType = DX_SOUNDDATATYPE_FILE ;
		Movie->PF.MFSoundHandle = LoadSoundMemBase_UseGParam( &GParam, Movie->PF.FileName, 1, -1, FALSE, ASyncThread ) ;
//	}
//	else
//	{
//		GParam.CreateSoundDataType = DX_SOUNDDATATYPE_MEMPRESS ;
//		Movie->PF.MFSoundHandle = LoadSoundMemByMemImageBase_UseGParam( &GParam, TRUE, -1, FileImage, ( int )FileImageSize, 1, -1, FALSE, ASyncThread ) ;
//	}
	Movie->PF.MFSoundTotalTime = NS_GetSoundTotalTime( Movie->PF.MFSoundHandle ) ;
	Movie->PF.MFSoundFrequency = NS_GetFrequencySoundMem( Movie->PF.MFSoundHandle ) ;

	// ���[�v�^�C�v�̌���( �����ق�����ɂ��� )
//	Movie->PF.MFLoopType = Movie->PF.MFSoundTotalTime > Movie->StopTime / 10000 ? 1 : 0 ;
	Movie->PF.MFLoopType = 0 ;

#else // DX_NON_SOUND
	// ���[�v�^�C�v�͓���f�[�^���킹
	Movie->PF.MFLoopType = 0 ;
#endif // DX_NON_SOUND

	// �ŏ��̃t���[�����Z�b�g�A�b�v����
	{
		// �C���[�W���Z�b�g�A�b�v����Ă���A�t���O��|��
		Movie->PF.MFBaseImageSetup = 0 ;
#ifndef DX_NON_FILTER
		Movie->PF.MFYUVGrHandleSetup = 0 ;

		if( GSYS.HardInfo.UseShader )
		{
			MediaFoundationMovie_SetupImage( Movie, 0, 1, ASyncThread ) ;
		}
		else
#endif // DX_NON_FILTER
		{
			MediaFoundationMovie_SetupImage( Movie, 1, 0, ASyncThread ) ;
		}
	}

	// ����I��
	return 0 ;

ERR :
	if( Movie->PF.pMFReader != NULL						){		Movie->PF.pMFReader->Release() ;						Movie->PF.pMFReader = NULL ; }
	if( Movie->PF.pMFMediaTypeVideoStream != NULL		){		Movie->PF.pMFMediaTypeVideoStream->Release() ;			Movie->PF.pMFMediaTypeVideoStream = NULL ; }
	if( Movie->PF.pMFMediaTypeOutputVideoStream != NULL	){		Movie->PF.pMFMediaTypeOutputVideoStream->Release() ;	Movie->PF.pMFMediaTypeOutputVideoStream = NULL ; }
	if( Movie->PF.MFImageBuffer != NULL					){		DXFREE( Movie->PF.MFImageBuffer ) ;						Movie->PF.MFImageBuffer = NULL ; }
	if( Movie->PF.MFYBuffer != NULL						){		DXFREE( Movie->PF.MFYBuffer ) ;							Movie->PF.MFYBuffer = NULL ; }
	if( Movie->PF.MFUVBuffer != NULL					){		DXFREE( Movie->PF.MFUVBuffer ) ;						Movie->PF.MFUVBuffer = NULL ; }

	return -1 ;
}

// �J�����g�t���[����RGB�摜���쐬����( �߂�l  1:�쐬���ꂽ  0:����Ȃ����� )
static int MediaFoundationMovie_SetupImage(
	MOVIEGRAPH * Movie,
	int BaseImage,
	int YUVGrHandle,
	int ASyncThread
)
{
	unsigned char *d, *ys, *uvs ;
	int i, j, r, g, b, ysadd, dadd, uvadd, w, h, dpitch, uvr, uvg, uvb, y, y2, y3, y4;
	HRESULT hr ;
	DWORD dwFlags = 0 ;
	D_IMFSample *pSample = NULL;
	D_IMFMediaBuffer *pBuffer = NULL;
	D_IMF2DBuffer *p2DBuffer = NULL ;
//	BYTE *pScanline ;
//	LONG Pitch ;
	BYTE *Buffer ;
	DWORD MaxLength ;
	DWORD CurrentLength ;
	LONGLONG llTimestamp ;

	// ���݂̃t���[���̉摜���Z�b�g�A�b�v����Ă���Ή��������I��
	if( 
#ifndef DX_NON_FILTER
		( ( YUVGrHandle && Movie->PF.MFYUVGrHandleSetup ) || YUVGrHandle == 0 || Movie->PF.MFNotUseYUVGrHandle == TRUE  ) &&
#endif // DX_NON_FILTER
		( ( BaseImage   && Movie->PF.MFBaseImageSetup   ) || BaseImage   == 0 ) )
		return 0 ;

	// �C���[�W�f�[�^�̓ǂݍ���
	if( Movie->PF.MFLastReadSampleFrame < Movie->PF.MFCurrentFrame )
	{
		for(;;)
		{
			int CurFrame ;

			if( pSample != NULL )
			{
				pSample->Release() ;
				pSample = NULL ;
			}

			hr = Movie->PF.pMFReader->ReadSample(
				( DWORD )D_MF_SOURCE_READER_FIRST_VIDEO_STREAM,
				0,
				NULL,
				&dwFlags,
				&llTimestamp,
				&pSample
			) ;
			if( hr != S_OK )
			{
				goto ERR ;
			}

			if( dwFlags & D_MF_SOURCE_READERF_CURRENTMEDIATYPECHANGED )
			{
				continue ;
			}

			if( dwFlags & D_MF_SOURCE_READERF_ENDOFSTREAM )
			{
				goto ERR ;
			}

			if( pSample == NULL )
			{
				goto ERR ;
			}

			// �t���[�����Z�o
			{
				LONGLONG Temp64_1, Temp64_2 ;
				DWORD Temp128[ 4 ] ;

				Temp64_1 = llTimestamp + Movie->PF.FrameTime / 4 ;
				Temp64_2 = ( LONGLONG )Movie->PF.MFFrameRateNumerator * 0x1000 / Movie->PF.MFFrameRateDenominator ;
				_MUL128_1( ( DWORD * )&Temp64_1, ( DWORD * )&Temp64_2, Temp128 ) ;

				Temp64_2 = LL_NUM( 40960000000 ) /* 10000000 * 0x1000 */ ;
				_DIV128_1( Temp128, ( DWORD * )&Temp64_2, ( DWORD * )&Temp64_1 ) ;

				CurFrame = ( int )Temp64_1 ;
			}

			Movie->PF.MFLastReadSampleTimeStamp = llTimestamp ;
			Movie->PF.MFLastReadSampleFrame     = CurFrame ;

			if( CurFrame >= Movie->PF.MFCurrentFrame )
			{
				break ;
			}
			else
			{
				CurFrame = CurFrame ;
			}
		}

		hr = pSample->GetBufferByIndex( 0, &pBuffer ) ;
		if( hr != S_OK )
		{
			goto ERR ;
		}

	//	hr = pBuffer->QueryInterface( IID_IMF2DBUFFER, ( void ** )&p2DBuffer ) ;
	//	if( hr != S_OK )
	//	{
	//		goto ERR ;
	//	}
	//
	//	hr = p2DBuffer->Lock2D( &pScanline, &Pitch ) ;
	//	if( hr != S_OK )
	//	{
	//		goto ERR ;
	//	}

		hr = pBuffer->Lock( &Buffer, &MaxLength, &CurrentLength ) ;
		if( hr != S_OK )
		{
			goto ERR ;
		}

		// �o�b�t�@�Ƀf�[�^���R�s�[����
		{
			BYTE *yimage ;
			BYTE *uimage ;
			BYTE *vimage ;
			DWORD uvstride ;

			uvstride = ( Movie->PF.MFFrameSizeX / 2 + 7 ) / 8 * 8 ;

			yimage = ( BYTE * )Buffer ;
			uimage = yimage + Movie->PF.MFYStride  * ( ( Movie->PF.MFFrameSizeY + 15 ) / 16 * 16 ) ;
			vimage = uimage + uvstride * ( ( Movie->PF.MFFrameSizeY / 2 + 7 ) / 8 * 8 ) ;
			_MEMCPY( Movie->PF.MFYBuffer, yimage, ( size_t )( Movie->PF.MFYStride * Movie->PF.MFFrameSizeY ) ) ;
			{
				DWORD n, m ;
				DWORD bw ;
				DWORD ow ;
				BYTE *dst ;
				BYTE *u, *v ;

				bw = Movie->PF.MFUVWidth / 8 ;
				ow = Movie->PF.MFUVWidth % 8 ;

				dst = ( BYTE * )Movie->PF.MFUVBuffer ;
				u = uimage ;
				v = vimage ;
				for( n = 0 ; n < ( DWORD )Movie->PF.MFUVHeight ; n ++, dst += Movie->PF.MFUVStride, u += uvstride, v += uvstride )
				{
					BYTE *u_t, *v_t ;
					BYTE *dst_t ;

					dst_t = dst ;
					u_t = u ;
					v_t = v ;
					for( m = 0 ; m < bw ; m ++, dst_t += 2 * 8, u_t += 8, v_t += 8 )
					{
						dst_t[  0 ] = u_t[ 0 ] ;		dst_t[  1 ] = v_t[ 0 ] ;
						dst_t[  2 ] = u_t[ 1 ] ;		dst_t[  3 ] = v_t[ 1 ] ;
						dst_t[  4 ] = u_t[ 2 ] ;		dst_t[  5 ] = v_t[ 2 ] ;
						dst_t[  6 ] = u_t[ 3 ] ;		dst_t[  7 ] = v_t[ 3 ] ;
						dst_t[  8 ] = u_t[ 4 ] ;		dst_t[  9 ] = v_t[ 4 ] ;
						dst_t[ 10 ] = u_t[ 5 ] ;		dst_t[ 11 ] = v_t[ 5 ] ;
						dst_t[ 12 ] = u_t[ 6 ] ;		dst_t[ 13 ] = v_t[ 6 ] ;
						dst_t[ 14 ] = u_t[ 7 ] ;		dst_t[ 15 ] = v_t[ 7 ] ;
					}
					for( m = 0 ; m < ow ; m ++, dst_t += 2, u_t ++, v_t ++ )
					{
						dst_t[ 0 ] = *u_t ;
						dst_t[ 1 ] = *v_t ;
					}
				}
			}
		}

		if( pBuffer->Unlock() != S_OK )
		{
			goto ERR ;
		}
	}

#ifndef DX_NON_FILTER
	// �V�F�[�_�[���g�p�ł���ꍇ��YUV�J���[�̃O���t�B�b�N�n���h�����g�p����
	if( YUVGrHandle && Movie->PF.MFNotUseYUVGrHandle == FALSE && GetValidShaderVersion() >= 200 )
	{
		SETUP_GRAPHHANDLE_GPARAM GParam ;
		BASEIMAGE BaseImage ;
		RECT SrcRect ;

		// �O���t�B�b�N�n���h�����܂��쐬����Ă��Ȃ�������쐬����
		if( Movie->YGrHandle == -1 )
		{
			Graphics_Image_InitSetupGraphHandleGParam( &GParam ) ;

			Graphics_Image_InitSetupGraphHandleGParam_Normal_NonDrawValid( &GParam, 32, FALSE ) ;
			GParam.CreateImageChannelNum      = 1 ;
			GParam.CreateImageChannelBitDepth = 8 ;
			Movie->YGrHandle = Graphics_Image_MakeGraph_UseGParam( &GParam, Movie->PF.MFYWidth, Movie->PF.MFYHeight, FALSE, FALSE, 0, FALSE, ASyncThread ) ;
			NS_SetDeleteHandleFlag( Movie->YGrHandle, ( int * )&Movie->YGrHandle ) ;
		}
		if( Movie->UVGrHandle == -1 )
		{
			Graphics_Image_InitSetupGraphHandleGParam( &GParam ) ;

			Graphics_Image_InitSetupGraphHandleGParam_Normal_NonDrawValid( &GParam, 32, FALSE ) ;
			GParam.CreateImageChannelNum      = 2 ;
			GParam.CreateImageChannelBitDepth = 8 ;
			Movie->UVGrHandle     = Graphics_Image_MakeGraph_UseGParam( &GParam, Movie->PF.MFUVWidth, Movie->PF.MFUVHeight, FALSE, FALSE, 0, FALSE, ASyncThread ) ;
			NS_SetDeleteHandleFlag( Movie->UVGrHandle, ( int * )&Movie->UVGrHandle ) ;
		}

		_MEMSET( &BaseImage, 0, sizeof( BaseImage ) ) ;
		BaseImage.Width                     = Movie->PF.MFYWidth ;
		BaseImage.Height                    = Movie->PF.MFYHeight ;
		BaseImage.Pitch                     = Movie->PF.MFYStride ;
		BaseImage.GraphData                 = Movie->PF.MFYBuffer ;
		BaseImage.ColorData.Format          = DX_BASEIMAGE_FORMAT_NORMAL ;
		BaseImage.ColorData.PixelByte       = 1 ;
		BaseImage.ColorData.ChannelNum      = 1 ;
		BaseImage.ColorData.ChannelBitDepth = 8 ;
		SETRECT( SrcRect, 0, 0, BaseImage.Width, BaseImage.Height ) ;
		Graphics_Image_BltBmpOrBaseImageToGraph3(
			&SrcRect,
			0,
			0,
			Movie->YGrHandle,
			&BaseImage,
			NULL,
			FALSE,
			FALSE,
			FALSE,
			ASyncThread
		) ;

		BaseImage.Width                     = Movie->PF.MFUVWidth ;
		BaseImage.Height                    = Movie->PF.MFUVHeight ;
		BaseImage.Pitch                     = Movie->PF.MFUVStride ;
		BaseImage.GraphData                 = Movie->PF.MFUVBuffer ;
		BaseImage.ColorData.Format          = DX_BASEIMAGE_FORMAT_NORMAL ;
		BaseImage.ColorData.PixelByte       = 2 ;
		BaseImage.ColorData.ChannelNum      = 2 ;
		BaseImage.ColorData.ChannelBitDepth = 8 ;
		SETRECT( SrcRect, 0, 0, BaseImage.Width, BaseImage.Height ) ;
		Graphics_Image_BltBmpOrBaseImageToGraph3(
			&SrcRect,
			0,
			0,
			Movie->UVGrHandle,
			&BaseImage,
			NULL,
			FALSE,
			FALSE,
			FALSE,
			ASyncThread
		) ;

		// �Z�b�g�A�b�v�t���O�𗧂Ă�
		Movie->PF.MFYUVGrHandleSetup = 1 ;
	}
#endif // DX_NON_FILTER

	// �q�f�a�C���[�W�̃Z�b�g�A�b�v�w�肪�����āA�܂��쐬����Ă��Ȃ�������쐬����
	if( BaseImage && Movie->PF.MFBaseImageSetup == 0 )
	{
		// BASEIMAGE �̏ꍇ
		d  = ( unsigned char * )Movie->NowImage.GraphData ;
		ys = ( unsigned char * )Movie->PF.MFYBuffer ;
		uvs = ( unsigned char * )Movie->PF.MFUVBuffer ;

		// yuv ���� rgb �f�[�^�ɕϊ�
		if( Movie->PF.MFYWidth  == Movie->PF.MFUVWidth  * 2 &&
			Movie->PF.MFYHeight == Movie->PF.MFUVHeight * 2 )
		{
			ysadd  = Movie->PF.MFYStride  * 2 - Movie->PF.MFYWidth ;
			uvadd  = Movie->PF.MFUVStride     - Movie->PF.MFUVWidth * 2 ;
			dadd   = Movie->NowImage.Pitch * 2 - Movie->PF.MFYWidth * 4 ;
			dpitch = Movie->NowImage.Pitch ;
			w = Movie->PF.MFYWidth  / 2 ;
			h = Movie->PF.MFYHeight / 2 ;
			for( i = 0; i < h; i++, d += dadd, ys += ysadd, uvs += uvadd )
			{
				for( j = 0; j < w; j ++, d += 8, ys += 2, uvs += 2 )
				{
					uvr =                              YUVTable[ YUV_RV ][ uvs[ 1 ] ] ;
					uvg = YUVTable[ YUV_GU ][ uvs[ 0 ] ] + YUVTable[ YUV_GV ][ uvs[ 1 ] ] ;
					uvb = YUVTable[ YUV_BU ][ uvs[ 0 ] ]                              ;

					y  = YUVTable[ YUV_Y ][ ys[ 0 ] ] ;
					y2 = YUVTable[ YUV_Y ][ ys[ 1 ] ] ;
					y3 = YUVTable[ YUV_Y ][ ys[ Movie->PF.MFYStride ] ] ;
					y4 = YUVTable[ YUV_Y ][ ys[ Movie->PF.MFYStride + 1 ] ] ;

					d[2]              = YUVLimitTable[ ( ( y  + uvr ) >> 14 ) + 512 ] ;
					d[1]              = YUVLimitTable[ ( ( y  + uvg ) >> 14 ) + 512 ] ;
					d[0]              = YUVLimitTable[ ( ( y  + uvb ) >> 14 ) + 512 ] ;
					d[3]              = 255;

					d[2 + 4]          = YUVLimitTable[ ( ( y2 + uvr ) >> 14 ) + 512 ] ;
					d[1 + 4]          = YUVLimitTable[ ( ( y2 + uvg ) >> 14 ) + 512 ] ;
					d[0 + 4]          = YUVLimitTable[ ( ( y2 + uvb ) >> 14 ) + 512 ] ;
					d[3 + 4]          = 255;

					d[2 + dpitch]     = YUVLimitTable[ ( ( y3 + uvr ) >> 14 ) + 512 ] ;
					d[1 + dpitch]     = YUVLimitTable[ ( ( y3 + uvg ) >> 14 ) + 512 ] ;
					d[0 + dpitch]     = YUVLimitTable[ ( ( y3 + uvb ) >> 14 ) + 512 ] ;
					d[3 + dpitch]     = 255;

					d[2 + dpitch + 4] = YUVLimitTable[ ( ( y4 + uvr ) >> 14 ) + 512 ] ;
					d[1 + dpitch + 4] = YUVLimitTable[ ( ( y4 + uvg ) >> 14 ) + 512 ] ;
					d[0 + dpitch + 4] = YUVLimitTable[ ( ( y4 + uvb ) >> 14 ) + 512 ] ;
					d[3 + dpitch + 4] = 255;
				}
			}
		}
		else
		{
			ysadd = Movie->PF.MFYStride - Movie->PF.MFYWidth;
			dadd  = Movie->NowImage.Pitch - Movie->PF.MFYWidth * 4;
			for( i = 0; ( DWORD )i < Movie->PF.MFYHeight; i++, d += dadd, ys += ysadd )
			{
				for( j = 0; ( DWORD )j < Movie->PF.MFYWidth; j ++, d += 4, ys++ )
				{
					uvs = ( BYTE * )Movie->PF.MFUVBuffer + ( i / 2 ) * Movie->PF.MFUVStride + j / 2 * 2 ;

					r = _FTOL(1.164f * (*ys-16)                         + 1.596f * (uvs[1]-128));
					g = _FTOL(1.164f * (*ys-16) - 0.391f * (uvs[0]-128) - 0.813f * (uvs[1]-128));
					b = _FTOL(1.164f * (*ys-16) + 2.018f * (uvs[0]-128));
					if( r < 0 ) r = 0; else if( r > 255 ) r = 255;
					if( g < 0 ) g = 0; else if( g > 255 ) g = 255;
					if( b < 0 ) b = 0; else if( b > 255 ) b = 255;
					d[2] = ( unsigned char )r;
					d[1] = ( unsigned char )g;
					d[0] = ( unsigned char )b;
					d[3] = 255;
				}
			}
		}

		// �Z�b�g�A�b�v�t���O�𗧂Ă�
		Movie->PF.MFBaseImageSetup = 1 ;
	}

	if( pSample != NULL )
	{
		pSample->Release() ;
		pSample = NULL ;
	}

	if( p2DBuffer != NULL )
	{
		p2DBuffer->Release() ;
		p2DBuffer = NULL ;
	}

	if( pBuffer != NULL )
	{
		pBuffer->Release() ;
		pBuffer = NULL ;
	}

	// ����I��
	return 0 ;

ERR :
	if( pSample != NULL )
	{
		pSample->Release() ;
		pSample = NULL ;
	}

	if( p2DBuffer != NULL )
	{
		p2DBuffer->Release() ;
		p2DBuffer = NULL ;
	}

	if( pBuffer != NULL )
	{
		pBuffer->Release() ;
		pBuffer = NULL ;
	}

	return -1 ;
}

// Media Foundation ���g�p��������t�@�C���̃t���[����i�߂�
static int MediaFoundationMovie_IncToFrame(
	MOVIEGRAPH * Movie,
	int AddFrame
)
{
	// �i�߂�t���[������ 0 �̏ꍇ�͉��������I��
	if( AddFrame <= 0 )
	{
		return 0 ;
	}

	// �J�����g�t���[����ύX
	Movie->PF.MFCurrentFrame += AddFrame ;

	// �v�������t���[�����f�R�[�h�ς݂̃t���[�����ԍ����Ⴂ�ꍇ�̓Z�b�g�A�b�v�t���O��|���Ȃ�
	if( Movie->PF.MFLastReadSampleFrame < Movie->PF.MFCurrentFrame )
	{
		// �C���[�W���Z�b�g�A�b�v����Ă���A�t���O��|��
		Movie->PF.MFBaseImageSetup = 0 ;
#ifndef DX_NON_FILTER
		Movie->PF.MFYUVGrHandleSetup = 0 ;

		if( GSYS.HardInfo.UseShader )
		{
			MediaFoundationMovie_SetupImage( Movie, 0, 1, FALSE ) ;
		}
		else
#endif // DX_NON_FILTER
		{
			MediaFoundationMovie_SetupImage( Movie, 1, 0, FALSE ) ;
		}
	}

	// ����I��
	return 0 ;
}

#endif // DX_NON_MEDIA_FOUNDATION

#ifndef DX_NON_DSHOW_MOVIE

// DirectShow ���g�p��������t�@�C���̃I�[�v�������݂�
extern int DirectShowOpenMovieFile(
	MOVIEGRAPH * Movie,
	const wchar_t *FileName,
	int *Width,
	int *Height,
	int SurfaceMode,
	int ASyncThread
)
{
	wchar_t ErStr[256] ;
	D_IAMMultiMediaStream *pAMStream = NULL;
	HRESULT hr ;

	Movie->NowImage.GraphData = NULL;
	Movie->YUVFlag = FALSE ;
	Movie->OverlayDestX = 0 ;
	Movie->OverlayDestY = 0 ;
	Movie->OverlayDestExRate = 0 ;
	Movie->OverlayDispFlag = 0 ;
	Movie->FirstUpdateFlag = FALSE ;
	_MEMSET( &Movie->OverlaySrcRect, 0, sizeof( RECT ) ) ;
	_MEMSET( &Movie->OverlayDestRect, 0, sizeof( RECT ) ) ;

	// �����I�[�o�[���C���g���Ȃ��ꍇ�̓t���J���[�ɂ���
	if( SurfaceMode == DX_MOVIESURFACE_OVERLAY  )
	{
		SurfaceMode = DX_MOVIESURFACE_FULLCOLOR ;
	}

	// ������ʂ��R�Q�r�b�g�J���[���[�h�Ńt���J���[���[�h���w�肵�Ă����ꍇ�̓m�[�}���ɂ���
	if( SurfaceMode == DX_MOVIESURFACE_FULLCOLOR && NS_GetColorBitDepth() == 32 ) SurfaceMode = DX_MOVIESURFACE_NORMAL ;

	SurfaceMode = DX_MOVIESURFACE_FULLCOLOR ;

	// �O���t�B�b�N�r���_�[�I�u�W�F�N�g�̍쐬
	if( ( FAILED( WinAPI_CoCreateInstance_ASync(CLSID_FILTERGRAPH, NULL, CLSCTX_INPROC, IID_IGRAPHBUILDER, (void **)&Movie->PF.pGraph, ASyncThread ) ) ) )
	{
		_WCSCPY_S( ErStr, sizeof( ErStr ), L"CoCreateInstance Error : CLSID_FilterGraph\n" ) ;
		goto ERROR_R ;
	}

	// Create the Texture Renderer object
	Movie->PF.pMovieImage = New_D_CMovieRender( NULL, &hr ) ;
    
    // Get a pointer to the IBaseFilter on the TextureRenderer, add it to graph
	hr = Movie->PF.pGraph->AddFilter( Movie->PF.pMovieImage, L"MovieRenderer" ) ;
    if( FAILED( hr ) )
    {
        _WCSCPY_S( ErStr, sizeof( ErStr ), L"Could not add renderer filter to graph!\n" ) ;
        return hr;
    }

	// BasicAudio �C���^�[�t�F�C�X�𓾂�
	if( FAILED( Movie->PF.pGraph->QueryInterface( IID_IBASICAUDIO, ( void ** )&Movie->PF.pBasicAudio ) ) )
	{
		_WCSCPY_S( ErStr, sizeof( ErStr ), L"QueryInterface Error : IID_IBasicAudio\n" ) ;
		goto ERROR_R ;
	}

	// ���f�B�A�R���g���[���I�u�W�F�N�g���擾����
	if( FAILED( Movie->PF.pGraph->QueryInterface( IID_IMEDIACONTROL, ( void ** )&Movie->PF.pMediaControl ) ) )
	{
		_WCSCPY_S( ErStr, sizeof( ErStr ), L"QueryInterface Error : IID_IMediaControl\n" ) ;
		goto ERROR_R ;
	}

	// ���f�B�A�V�[�L���O�I�u�W�F�N�g���擾����
	if( FAILED( Movie->PF.pGraph->QueryInterface( IID_IMEDIASEEKING, ( void ** )&Movie->PF.pMediaSeeking ) ) )
	{
		_WCSCPY_S( ErStr, sizeof( ErStr ), L"QueryInterface Error : IID_IMediaSeeking\n" ) ;
		goto ERROR_R ;
	}

   	// �t�@�C�����ۑ�
	_WCSCPY_S( Movie->PF.FileName, sizeof( Movie->PF.FileName ), FileName ) ;

	// �f�B���N�g���̋�؂肪 / �ɂȂ��Ă���ӏ��� \ �ɒu��������
	{
		wchar_t *wp ;

		for( wp = Movie->PF.FileName; *wp; wp++ )
		{
			if( *wp == L'/' )
			{
				*wp = '\\' ;
			}
		}
	}

	hr = Movie->PF.pGraph->RenderFile( Movie->PF.FileName, NULL ) ;
	if( FAILED( hr ) )
	{
		_WCSCPY_S( ErStr, sizeof( ErStr ), L"RenderFile faired!\n" ) ;

		DWORD_PTR fp ;
		HANDLE FileHandle ;
		void *TempBuffer ;
		size_t CompSize, MoveSize ;
		DWORD WriteSize ;
		size_t FileSize ;
		const DWORD BufferSize = 0x100000 ;

		_WCSCPY_S( ErStr, sizeof( ErStr ), L"Movie File Open Error : " ) ;
		_WCSCAT_S( ErStr, sizeof( ErStr ), FileName ) ;
		_WCSCAT_S( ErStr, sizeof( ErStr ), L"\n" ) ;

		// ���Ƀe���|�����t�@�C�����쐬���Ă���ꍇ�́A
		// �X�Ƀe���|�����t�@�C�����쐬���邱�Ƃ͂��Ȃ�
		if( Movie->PF.UseTemporaryFile == TRUE )
			goto ERROR_R ;

		// �t�@�C�����J���Ȃ�������A�[�J�C�u����Ă���\��������
#ifdef UNICODE
		fp = DX_FOPEN( Movie->PF.FileName ) ;
#else
		fp = DX_FOPEN( FileName ) ;
#endif
		if( fp == 0 ) goto ERROR_R ;
		
		// �J�����ꍇ�̓e���|�����t�@�C���ɏ����o��
		{
			// �t�@�C���T�C�Y�̎擾
			DX_FSEEK( fp, 0L, SEEK_END ) ;
			FileSize = ( size_t )DX_FTELL( fp ) ;
			DX_FSEEK( fp, 0L, SEEK_SET ) ;

			// �ꎞ�I�Ƀf�[�^���i�[����o�b�t�@���m��
			TempBuffer = DXALLOC( BufferSize ) ;
			if( TempBuffer == NULL )
			{
				DX_FCLOSE( fp ) ;
				goto ERROR_R ;
			}

			// �e���|�����t�@�C���̍쐬
			FileHandle = CreateTemporaryFile( Movie->PF.FileName, sizeof( Movie->PF.FileName ) ) ;

			if( FileHandle == NULL )
			{
				DX_FCLOSE( fp ) ;
				DXFREE( TempBuffer ) ;
				goto ERROR_R ;
			}
			Movie->PF.UseTemporaryFile = TRUE ;

			// �e���|�����t�@�C���Ƀf�[�^�������o��
			CompSize = 0 ;
			while( CompSize < FileSize )
			{
				MoveSize = CompSize - FileSize ;
				if( MoveSize > BufferSize ) MoveSize = BufferSize ;

				DX_FREAD( TempBuffer, MoveSize, 1, fp ) ;
				WriteFile( FileHandle, TempBuffer, ( DWORD )MoveSize, &WriteSize, NULL ) ;

				if( MoveSize != WriteSize ) break ;
				CompSize += MoveSize ;
			}

			// �t�@�C������A���������������
			DX_FCLOSE( fp ) ;
			CloseHandle( FileHandle ) ;
			DXFREE( TempBuffer ) ;
		}

		// ���߂ăt�@�C�����J��
		hr = Movie->PF.pGraph->RenderFile( Movie->PF.FileName, NULL ) ;
		if( FAILED( hr ) )
		{
			// �e���|�����t�@�C�����폜
			DeleteFileW( Movie->PF.FileName ) ;
			goto ERROR_R ;
		}
	}

	// �P�t���[��������̎��Ԃ𓾂�
	Movie->PF.pMediaSeeking->GetDuration( &Movie->PF.FrameTime ) ;
	if( Movie->PF.FrameTime == 0 )
	{
		Movie->PF.FrameTime = 10000000 / 60 ;
	}

	// �I�����Ԃ��擾����
	Movie->PF.pMediaSeeking->GetStopPosition( &Movie->StopTime ) ;

	// �摜�C���[�W�̏����Z�b�g����
	Movie->NowImage.Width        = ( int )Movie->PF.pMovieImage->Width ;
	Movie->NowImage.Height       = ( int )Movie->PF.pMovieImage->Height ;
	Movie->NowImage.Pitch        = ( int )Movie->PF.pMovieImage->Pitch ;
	Movie->NowImage.GraphData    = Movie->PF.pMovieImage->ImageBuffer ;
	Movie->NowImageGraphOutAlloc = TRUE ;
	if( Movie->PF.pMovieImage->ImageType == 0 )
	{
		NS_CreateFullColorData( &Movie->NowImage.ColorData ) ;
	}
	else
	if( Movie->PF.pMovieImage->ImageType == 1 && Movie->A8R8G8B8Flag )
	{
		NS_CreateARGB8ColorData( &Movie->NowImage.ColorData ) ;
	}
	else
	{
		NS_CreateXRGB8ColorData( &Movie->NowImage.ColorData ) ;
	}
	Movie->UseNowImage = &Movie->NowImage ;

	if( Width  ) *Width  = ( int )Movie->PF.pMovieImage->Width ;
	if( Height ) *Height = ( int )Movie->PF.pMovieImage->Height ;

	// �I��
	return 0 ;


ERROR_R:

	// �e��b�n�l�I�u�W�F�N�g���I������
	if( pAMStream					){ pAMStream->Release()					; pAMStream = NULL ; }

	if( Movie->PF.pGraph			){ Movie->PF.pGraph->Release()			; Movie->PF.pGraph = NULL ; }
	if( Movie->PF.pMediaControl		){ Movie->PF.pMediaControl->Release()	; Movie->PF.pMediaControl = NULL ; }
	if( Movie->PF.pMediaSeeking		){ Movie->PF.pMediaSeeking->Release()	; Movie->PF.pMediaSeeking = NULL ; }
	if( Movie->PF.pBasicAudio		){ Movie->PF.pBasicAudio->Release()		; Movie->PF.pBasicAudio = NULL ; }

	return DXST_LOGFILE_ADDW( ErStr ) ;
}

#endif // DX_NON_DSHOW_MOVIE


// ���[�r�[�n���h���̌�n�����s��
extern int TerminateMovieHandle_PF( HANDLEINFO *HandleInfo )
{
	MOVIEGRAPH *Movie = ( MOVIEGRAPH * )HandleInfo ;

#if !defined( DX_NON_MEDIA_FOUNDATION ) || !defined( DX_NON_DSHOW_MOVIE )
	// �����e���|�����t�@�C�����g�p���Ă����ꍇ�̓e���|�����t�@�C�����폜����
	if( Movie->PF.UseTemporaryFile == TRUE )
	{
		DeleteFileW( Movie->PF.FileName ) ;
		Movie->PF.UseTemporaryFile = FALSE ;
	}
#endif // !defined( DX_NON_MEDIA_FOUNDATION ) || !defined( DX_NON_DSHOW_MOVIE )

#ifndef DX_NON_MEDIA_FOUNDATION
#ifndef DX_NON_SOUND
	if( Movie->PF.MFSoundHandle > 0 )
	{
		NS_DeleteSoundMem( Movie->PF.MFSoundHandle ) ;
		Movie->PF.MFSoundHandle = 0 ;
	}
#endif // DX_NON_SOUND

	if( Movie->PF.pMFReader != NULL						){		Movie->PF.pMFReader->Release() ;						Movie->PF.pMFReader = NULL ; }
	if( Movie->PF.pMFMediaTypeVideoStream != NULL		){		Movie->PF.pMFMediaTypeVideoStream->Release() ;			Movie->PF.pMFMediaTypeVideoStream = NULL ; }
	if( Movie->PF.pMFMediaTypeOutputVideoStream != NULL	){		Movie->PF.pMFMediaTypeOutputVideoStream->Release() ;	Movie->PF.pMFMediaTypeOutputVideoStream = NULL ; }
	if( Movie->PF.MFImageBuffer != NULL					){		DXFREE( Movie->PF.MFImageBuffer ) ;						Movie->PF.MFImageBuffer = NULL ; }
	if( Movie->PF.MFYBuffer != NULL						){		DXFREE( Movie->PF.MFYBuffer ) ;							Movie->PF.MFYBuffer = NULL ; }
	if( Movie->PF.MFUVBuffer != NULL					){		DXFREE( Movie->PF.MFUVBuffer ) ;						Movie->PF.MFUVBuffer = NULL ; }
#endif // DX_NON_MEDIA_FOUNDATION

#ifndef DX_NON_DSHOW_MOVIE
	if( Movie->PF.pBasicAudio )		{ Movie->PF.pBasicAudio->Release() ; 		Movie->PF.pBasicAudio = NULL ; }
	if( Movie->PF.pMediaSeeking )	{ Movie->PF.pMediaSeeking->Release() ; 		Movie->PF.pMediaSeeking = NULL ; }
	if( Movie->PF.pMediaControl )	{ Movie->PF.pMediaControl->Release(); 		Movie->PF.pMediaControl = NULL ; }
	if( Movie->PF.pGraph )			{ Movie->PF.pGraph->Release(); 				Movie->PF.pGraph = NULL ; }
	if( Movie->PF.pMovieImage )		{ delete Movie->PF.pMovieImage ;			Movie->PF.pMovieImage = NULL ; }
#endif // DX_NON_DSHOW_MOVIE

	// ����I��
	return 0 ;
}


// OpenMovie �̃O���[�o���ϐ��ɃA�N�Z�X���Ȃ��o�[�W�����̊��ˑ�����
extern int OpenMovie_UseGParam_PF( MOVIEGRAPH * Movie, OPENMOVIE_GPARAM *GParam, const wchar_t *FileName, int *Width, int *Height, int SurfaceMode, int ASyncThread )
{
	int Flag = FALSE ;

#ifndef DX_NON_MEDIA_FOUNDATION
	// Media Foundation �ɂ��t�@�C���̃I�[�v�������݂�
	if( Flag == FALSE )
	{
		Movie->PF.UseTemporaryFile = FALSE ;
		if( MediaFoundationOpenMovieFile( Movie, FileName, Width, Height, ASyncThread ) != -1 )
		{
			Flag = TRUE ;
		}
	}
#endif // DX_NON_MEDIA_FOUNDATION

#ifndef DX_NON_DSHOW_MOVIE
	// DirectShow �ɂ��t�@�C���̃I�[�v�������݂�
	if( Flag == FALSE )
	{
		Movie->PF.UseTemporaryFile = FALSE ;
		if( DirectShowOpenMovieFile( Movie, FileName, Width, Height,SurfaceMode, ASyncThread ) != -1 )
		{
			Flag = TRUE ;
		}
	}
#endif // DX_NON_DSHOW_MOVIE

	if( Flag == FALSE )
	{
		return -1 ;
	}

	// ����
	return 0 ;
}

// ���[�r�[�̍Đ����J�n���鏈���̊��ˑ�����
extern int PlayMovie__PF( MOVIEGRAPH * Movie, int PlayType, int SysPlay )
{
#ifndef DX_NON_MEDIA_FOUNDATION
	if( Movie->PF.pMFReader != NULL )
	{
		// �P�t���[���ڂ��Z�b�g�A�b�v
#ifndef DX_NON_FILTER
		if( GSYS.HardInfo.UseShader )
		{
			MediaFoundationMovie_SetupImage( Movie, 0, 1, FALSE ) ;
		}
		else
#endif // DX_NON_FILTER
		{
			MediaFoundationMovie_SetupImage( Movie, 1, 0, FALSE ) ;
		}

		// �T�E���h�̍Đ����J�n����
#ifndef DX_NON_SOUND
		NS_PlaySoundMem( Movie->PF.MFSoundHandle, Movie->PF.MFLoopType == 1 ? PlayType : DX_PLAYTYPE_BACK, FALSE ) ;
#endif // DX_NON_SOUND

		// �Đ��J�n���̎��Ԃ��擾
		Movie->PF.MFPrevTimeCount = NS_GetNowHiPerformanceCount() ;

		// �Đ����Ԃ��Z�b�g
		Movie->PF.MFPlayNowTime = Movie->PF.MFFrameRateDenominator * ( LONGLONG )Movie->PF.MFCurrentFrame * ( LONGLONG )1000000 / Movie->PF.MFFrameRateNumerator ;

		// ����I��
		return 0 ;
	}
#endif // DX_NON_MEDIA_FOUNDATION

#ifndef DX_NON_DSHOW_MOVIE
	// DirectShow �ɂ��Đ����J�n
	if( Movie->PF.pMediaControl != NULL )
	{
		Movie->PF.pMediaControl->Run() ;
	}
#endif // DX_NON_DSHOW_MOVIE

	// ����I��
	return 0 ;
}

// ���[�r�[�̍Đ����X�g�b�v���鏈���̊��ˑ�����
extern 	int PauseMovie_PF( MOVIEGRAPH * Movie, int SysPause )
{
#ifndef DX_NON_MEDIA_FOUNDATION
	if( Movie->PF.pMFReader != NULL )
	{
		// �T�E���h�̍Đ����~����
#ifndef DX_NON_SOUND
		NS_StopSoundMem( Movie->PF.MFSoundHandle ) ;
#endif // DX_NON_SOUND

		// ���݂̍Đ����ԕ��܂Ńt���[����i�߂Ă���
		UpdateMovie( Movie->HandleInfo.Handle ) ;

		return 0 ;
	}
#endif // DX_NON_MEDIA_FOUNDATION


#ifndef DX_NON_DSHOW_MOVIE
	if( Movie->PF.pMediaControl != NULL )
	{
		// ��~
		Movie->PF.pMediaControl->Pause() ;

		// ����I��
		return 0 ;
	}
#endif // DX_NON_DSHOW_MOVIE

	return -1 ; 
}

// ���[�r�[�̍Đ��ʒu��ݒ肷��(�~���b�P��)�����̊��ˑ�����
extern int SeekMovie_PF( MOVIEGRAPH * Movie, int Time )
{
#ifndef DX_NON_MEDIA_FOUNDATION
	if( Movie->PF.pMFReader != NULL )
	{
		D_PROPVARIANT variant ;

		_MEMSET( &variant, 0, sizeof( variant ) ) ;
		variant.vt = D_VT_I8 ;
		variant.hVal.QuadPart = ( ULONGLONG )Time * 10000 ;
		Movie->PF.pMFReader->SetCurrentPosition( _GUID_NULL, variant ) ;

		// �Đ��t���[�����Z�o
		{
			LONGLONG Temp64_1, Temp64_2 ;
			DWORD Temp128[ 4 ] ;

			Temp64_1 = ( LONGLONG )Time * 1000 ;
			Temp64_2 = ( LONGLONG )Movie->PF.MFFrameRateNumerator * 0x10000 / Movie->PF.MFFrameRateDenominator ;
			_MUL128_1( ( DWORD * )&Temp64_1, ( DWORD * )&Temp64_2, Temp128 ) ;

			Temp64_2 = LL_NUM( 65536000000 ) /* 1000000 * 0x10000 */ ;
			_DIV128_1( Temp128, ( DWORD * )&Temp64_2, ( DWORD * )&Temp64_1 ) ;

			Movie->PF.MFCurrentFrame = ( int )Temp64_1 ;
		}

		// �Đ��J�n�^�C����ύX����
		Movie->PF.MFPlayNowTime = Movie->PF.MFFrameRateDenominator * ( LONGLONG )Movie->PF.MFCurrentFrame * ( LONGLONG )1000000 / Movie->PF.MFFrameRateNumerator ;

		// �Ō�� ReadSample �̎��ԁA�t���[�������Z�b�g
		Movie->PF.MFLastReadSampleTimeStamp = -1 ;
		Movie->PF.MFLastReadSampleFrame = -1 ;

		// �C���[�W���Z�b�g�A�b�v����Ă���A�t���O��|��
		Movie->PF.MFBaseImageSetup = 0 ;
#ifndef DX_NON_FILTER
		Movie->PF.MFYUVGrHandleSetup = 0 ;
#endif // DX_NON_FILTER
	
		// �Đ��ʒu��ύX����
#ifndef DX_NON_SOUND
		NS_SetSoundCurrentTime( ( int )( Movie->PF.MFPlayNowTime / 1000 ), Movie->PF.MFSoundHandle ) ;
#endif // DX_NON_SOUND

		// �V�[�N��̂P�t���[���ڂ��Z�b�g�A�b�v
#ifndef DX_NON_FILTER
		if( GSYS.HardInfo.UseShader )
		{
			MediaFoundationMovie_SetupImage( Movie, 0, 1, FALSE ) ;
		}
		else
#endif // DX_NON_FILTER
		{
			MediaFoundationMovie_SetupImage( Movie, 1, 0, FALSE ) ;
		}

		return 0 ;
	}
#endif // DX_NON_MEDIA_FOUNDATION


#ifndef DX_NON_DSHOW_MOVIE
	if( Movie->PF.pMediaSeeking != NULL )
	{
		LONGLONG Now, Stop ;

		Now = ( LONGLONG )Time * 10000 ;
		Stop = 0 ;
		Movie->PF.pMediaSeeking->SetPositions( &Now, D_AM_SEEKING_AbsolutePositioning, &Stop, D_AM_SEEKING_NoPositioning ) ;

		// ����I��
		return 0 ;
	}

#endif // DX_NON_DSHOW_MOVIE

	return -1 ;
}

// ���[�r�[�̍Đ����x��ݒ肷��( 1.0 = ���{��  2.0 = �Q�{�� )�����̊��ˑ�����
extern int SetPlaySpeedRateMovie_PF( MOVIEGRAPH * Movie, double SpeedRate )
{
#ifndef DX_NON_MEDIA_FOUNDATION
	if( Movie->PF.pMFReader != NULL )
	{
		Movie->PF.MFPlaySpeedRate = SpeedRate ;
#ifndef DX_NON_SOUND
		if( Movie->PF.MFSoundHandle != -1 )
		{
			NS_SetFrequencySoundMem( _DTOL( Movie->PF.MFSoundFrequency * Movie->PF.MFPlaySpeedRate ), Movie->PF.MFSoundHandle ) ;
		}
		else
#endif // DX_NON_SOUND
		{
		}

		return 0 ;
	}
#endif // DX_NON_MEDIA_FOUNDATION

#ifndef DX_NON_DSHOW_MOVIE
	if( Movie->PF.pMediaSeeking != NULL )
	{
		Movie->PF.pMediaSeeking->SetRate( SpeedRate ) ;

		// ����I��
		return 0 ;
	}

#endif // DX_NON_DSHOW_MOVIE

	return -1 ;
}

// ���[�r�[�̍Đ���Ԃ𓾂鏈���̊��ˑ�����
extern int GetMovieState_PF( MOVIEGRAPH * Movie )
{
#ifndef DX_NON_MEDIA_FOUNDATION
	if( Movie->PF.pMFReader != NULL )
	{
		return Movie->PlayFlag ;
	}
#endif // DX_NON_MEDIA_FOUNDATION

#ifndef DX_NON_DSHOW_MOVIE
	D_OAFilterState state ;
	LONGLONG Current ;

	if( Movie->PF.pMediaSeeking == NULL )
	{
		return Movie->PlayFlag ; 
	}

	if( Movie->PF.pMediaControl->GetState( 1000, &state ) != S_OK )
	{
		return Movie->PlayFlag ;
	}

	Movie->PF.pMediaSeeking->GetCurrentPosition( &Current ) ;
	if( ( Movie->PlayType & DX_PLAYTYPE_LOOPBIT ) == 0 && Current >= Movie->StopTime && state == D_State_Stopped )
	{
		Movie->PlayFlag = FALSE ;

		// �����ꎞ��~�t���O�𗧂Ă�
		Movie->SysPauseFlag = 1 ;
	}
#endif // DX_NON_DSHOW_MOVIE

	return Movie->PlayFlag ;
}

// ���[�r�[�̃{�����[�����Z�b�g����(0�`10000)�����̊��ˑ�����
extern int SetMovieVolume_PF( MOVIEGRAPH * Movie, int Volume )
{
	// ���ʕ␳
	if( Volume > 10000 )
	{
		Volume = 10000 ;
	}

	if( Volume < 0     )
	{
		Volume = 0 ;
	}

#ifndef DX_NON_MEDIA_FOUNDATION
	if( Movie->PF.pMFReader != NULL )
	{
		// �T�E���h�̉��ʂ��Z�b�g
#ifndef DX_NON_SOUND
		NS_SetVolumeSoundMem( Volume, Movie->PF.MFSoundHandle ) ;
#endif // DX_NON_SOUND

		return 0 ;
	}
#endif // DX_NON_MEDIA_FOUNDATION

#ifndef DX_NON_DSHOW_MOVIE
	if( Movie->PF.pBasicAudio != NULL )
	{
		Movie->PF.pBasicAudio->put_Volume( -10000 + Volume ) ;

		// ����I��
		return 0 ;
	}
#endif // DX_NON_DSHOW_MOVIE

	return -1 ;
}

// ���[�r�[�̊�{�C���[�W�f�[�^���擾���鏈���̊��ˑ�����
extern	BASEIMAGE *GetMovieBaseImage_PF( MOVIEGRAPH * Movie, int *ImageUpdateFlag, int ImageUpdateFlagSetOnly )
{
#ifndef DX_NON_MEDIA_FOUNDATION
	if( Movie->PF.pMFReader != NULL )
	{
		// ���[�r�[�̃t���[�����X�V
		UpdateMovie( Movie->HandleInfo.Handle ) ;

		// NowImage �̓��e���X�V���ꂽ���ǂ����̃t���O��������
		if( ImageUpdateFlag != NULL )
		{
			*ImageUpdateFlag = Movie->NowImageUpdateFlag ;
		}
		Movie->NowImageUpdateFlag = FALSE ;

		// ImageUpdateFlagSetOnly �� TRUE �̏ꍇ�͂����ŏI��
		if( ImageUpdateFlagSetOnly )
		{
			return NULL ;
		}

		// BaseImage ���Z�b�g�A�b�v����Ă��Ȃ�������Z�b�g�A�b�v����
		if( Movie->PF.MFBaseImageSetup == 0 )
		{
			MediaFoundationMovie_SetupImage( Movie, 1, 0, FALSE ) ;
		}

		// �A�h���X��Ԃ�
		return &Movie->NowImage ;
	}
#endif // DX_NON_MEDIA_FOUNDATION

#ifndef DX_NON_DSHOW_MOVIE
	if( Movie->PF.pMovieImage != NULL )
	{
		// ���[�r�[�̃t���[�����X�V
		if( GetMovieState( Movie->HandleInfo.Handle ) == FALSE )
		{
			int Time ;

			Time = TellMovie( Movie->HandleInfo.Handle ) ;
			PlayMovie_(  Movie->HandleInfo.Handle ) ;
			UpdateMovie( Movie->HandleInfo.Handle, TRUE ) ;
			PauseMovie(  Movie->HandleInfo.Handle ) ;
			SeekMovie(   Movie->HandleInfo.Handle, Time ) ;
		}
		else
		{
			if( ImageUpdateFlagSetOnly == FALSE && Movie->PF.pMovieImage->AlwaysBaseImage == 0 )
			{
				Movie->PF.pMovieImage->AlwaysBaseImage = 1 ;
			}

			UpdateMovie( Movie->HandleInfo.Handle ) ;
		}

		// NowImage �̓��e���X�V���ꂽ���ǂ����̃t���O��������
		if( ImageUpdateFlag != NULL )
		{
			*ImageUpdateFlag = Movie->NowImageUpdateFlag ;
		}
		Movie->NowImageUpdateFlag = FALSE ;

		return &Movie->NowImage ;
	}
#endif // DX_NON_DSHOW_MOVIE

	return NULL ;
}

// ���[�r�[�̑��t���[�����𓾂�
extern int GetMovieTotalFrame_PF( MOVIEGRAPH * Movie )
{
#ifndef DX_NON_MEDIA_FOUNDATION
	if( Movie->PF.pMFReader != NULL )
	{
		// ���t���[������Ԃ�
		return Movie->PF.MFTotalFrame ;
	}
#endif // DX_NON_MEDIA_FOUNDATION

	return -1 ;
}

// ���[�r�[�̍Đ��ʒu���擾����(�~���b�P��)�����̊��ˑ�����
extern int TellMovie_PF( MOVIEGRAPH * Movie )
{
#ifndef DX_NON_MEDIA_FOUNDATION
	if( Movie->PF.pMFReader != NULL )
	{
		// �t���[�����X�V
		UpdateMovie( Movie->HandleInfo.Handle ) ;

		// �t���[������Đ����Ԃ�����o��
		return ( int )( Movie->PF.MFFrameRateDenominator * Movie->PF.MFCurrentFrame * 1000 / Movie->PF.MFFrameRateNumerator ) ;
	}
#endif // DX_NON_MEDIA_FOUNDATION

#ifndef DX_NON_DSHOW_MOVIE
	if( Movie->PF.pMediaSeeking != NULL )
	{
		D_STREAM_TIME NowTime ;

		// ���Ԏ擾
		if( Movie->PF.pMediaSeeking->GetCurrentPosition( &NowTime ) != S_OK )
		{
			return -1 ;
		}

		// ���Ԃ�Ԃ�
		return _DTOL( (double)NowTime / 10000 ) ;
	}
#endif // DX_NON_DSHOW_MOVIE

	return -1 ;
}

// ���[�r�[�̍Đ��ʒu���擾����(�t���[���P��)�����̊��ˑ�����
extern int TellMovieToFrame_PF( MOVIEGRAPH * Movie )
{
#ifndef DX_NON_MEDIA_FOUNDATION
	if( Movie->PF.pMFReader != NULL )
	{
		// �t���[�����X�V
		UpdateMovie( Movie->HandleInfo.Handle ) ;

		// �t���[����Ԃ�
		return Movie->PF.MFCurrentFrame ;
	}
#endif // DX_NON_MEDIA_FOUNDATION

#ifndef DX_NON_DSHOW_MOVIE
	if( Movie->PF.pMediaSeeking != NULL )
	{
		D_STREAM_TIME NowTime ;

		// ���Ԏ擾
		if( Movie->PF.pMediaSeeking->GetCurrentPosition( &NowTime ) != S_OK )
		{
			return -1 ;
		}

		// ���Ԃ�Ԃ�
		return _DTOL( (double)NowTime / Movie->PF.FrameTime ) ;
	}
#endif // DX_NON_DSHOW_MOVIE

	return -1 ;
}

// ���[�r�[�̍Đ��ʒu��ݒ肷��(�t���[���P��)�����̊��ˑ�����
extern int SeekMovieToFrame_PF( MOVIEGRAPH * Movie, int Frame )
{
#ifndef DX_NON_MEDIA_FOUNDATION
	if( Movie->PF.pMFReader != NULL )
	{
		D_PROPVARIANT variant ;

		_MEMSET( &variant, 0, sizeof( variant ) ) ;
		variant.vt = D_VT_I8 ;
		variant.hVal.QuadPart = ( ULONGLONG )Movie->PF.MFFrameRateDenominator * ( ULONGLONG )Frame * ( ULONGLONG )10000000 / Movie->PF.MFFrameRateNumerator ;
		Movie->PF.pMFReader->SetCurrentPosition( _GUID_NULL, variant ) ;

		// �Đ��t���[�����Z�b�g
		Movie->PF.MFCurrentFrame = Frame ;

		// �Đ��J�n�^�C����ύX����
		Movie->PF.MFPlayNowTime = Movie->PF.MFFrameRateDenominator * ( LONGLONG )Movie->PF.MFCurrentFrame * ( LONGLONG )1000000 / Movie->PF.MFFrameRateNumerator ;

		// �Ō�� ReadSample �̎��ԁA�t���[�������Z�b�g
		Movie->PF.MFLastReadSampleTimeStamp = -1 ;
		Movie->PF.MFLastReadSampleFrame = -1 ;

		// �C���[�W���Z�b�g�A�b�v����Ă���A�t���O��|��
		Movie->PF.MFBaseImageSetup = 0 ;
#ifndef DX_NON_FILTER
		Movie->PF.MFYUVGrHandleSetup = 0 ;
#endif // DX_NON_FILTER

		// �Đ��ʒu��ύX����
#ifndef DX_NON_SOUND
		NS_SetSoundCurrentTime( ( int )( Movie->PF.MFPlayNowTime / 1000 ), Movie->PF.MFSoundHandle ) ;
#endif // DX_NON_SOUND

		return 0 ;
	}
#endif // DX_NON_MEDIA_FOUNDATION

#ifndef DX_NON_DSHOW_MOVIE
	if( Movie->PF.pMediaSeeking != NULL )
	{
		LONGLONG Now, Stop ;

		if( Movie->PF.pMediaSeeking == NULL ) return 0 ;

		Now = ( D_STREAM_TIME )_DTOL64( (double)Frame * Movie->PF.FrameTime ) ;
		Stop = 0 ;
		Movie->PF.pMediaSeeking->SetPositions( &Now, D_AM_SEEKING_AbsolutePositioning, &Stop, D_AM_SEEKING_NoPositioning ) ;

		// ����I��
		return 0 ;
	}
#endif // DX_NON_DSHOW_MOVIE

	return -1 ;
}

// ���[�r�[�̂P�t���[��������̎��Ԃ𓾂鏈���̊��ˑ�����
extern LONGLONG GetOneFrameTimeMovie_PF( MOVIEGRAPH * Movie )
{
#if !defined( DX_NON_DSHOW_MOVIE ) || !defined( DX_NON_MEDIA_FOUNDATION )

	return Movie->PF.FrameTime ;

#else // !defined( DX_NON_DSHOW_MOVIE ) || !defined( DX_NON_MEDIA_FOUNDATION )

	return -1 ;

#endif // !defined( DX_NON_DSHOW_MOVIE ) || !defined( DX_NON_MEDIA_FOUNDATION )
}





// ���[�r�[�̍X�V���s�������̊��ˑ�����
extern int UpdateMovie_PF( MOVIEGRAPH * Movie, int AlwaysFlag )
{
#ifndef DX_NON_MEDIA_FOUNDATION
	if( Movie->PF.pMFReader != NULL )
	{
		int NowFrame, AddFrame ;
		LONGLONG NowTime ;

		// ���̍Đ����ԕ��t���[����i�߂�
		if( Movie->SysPauseFlag == 0 )
		{
			LONGLONG Temp64_1, Temp64_2 ;
			DWORD Temp128[ 4 ] ;

			NowTime = NS_GetNowHiPerformanceCount() ;

			// �Đ��ςݎ��Ԃ�i�߂�
			if( Movie->PF.MFPlaySpeedRate < 0.999999999 || Movie->PF.MFPlaySpeedRate > 1.0000000001 )
			{
				Temp64_1 = NowTime - Movie->PF.MFPrevTimeCount ;
				Temp64_2 = _DTOL( Movie->PF.MFPlaySpeedRate * 0x10000 ) ;
				_MUL128_1( ( DWORD * )&Temp64_1, ( DWORD * )&Temp64_2, Temp128 ) ;

				Temp64_2 = 0x10000 ;
				_DIV128_1( Temp128, ( DWORD * )&Temp64_2, ( DWORD * )&Temp64_1 ) ;

				Movie->PF.MFPlayNowTime += Temp64_1 ;
			}
			else
			{
				Movie->PF.MFPlayNowTime += NowTime - Movie->PF.MFPrevTimeCount ;
			}

			Movie->PF.MFPrevTimeCount = NowTime ;

			// ���݂̃t���[���̎Z�o
			{
				Temp64_1 = Movie->PF.MFPlayNowTime ;
				Temp64_2 = ( LONGLONG )Movie->PF.MFFrameRateNumerator * 0x10000 / Movie->PF.MFFrameRateDenominator ;
				_MUL128_1( ( DWORD * )&Temp64_1, ( DWORD * )&Temp64_2, Temp128 ) ;

				Temp64_2 = LL_NUM( 65536000000 ) /* 1000000 * 0x10000 */ ;
				_DIV128_1( Temp128, ( DWORD * )&Temp64_2, ( DWORD * )&Temp64_1 ) ;

				NowFrame = ( int )Temp64_1 ;
			}

			// ���[�v�w�肪���邩�ǂ����ő��t���[�����𒴂��Ă���ꍇ�̏����𕪊򂷂�
			if( Movie->PF.MFTotalFrame <= NowFrame )
			{
				if( Movie->PlayType & DX_PLAYTYPE_LOOPBIT )
				{
					// ���[�v����ꍇ�͑��t���[�����Ŋ������]����o��
					NowFrame %= Movie->PF.MFTotalFrame ;
				}
				else
				{
					// ���[�v���Ȃ��ꍇ�͍ŏI�t���[���Ŏ~�܂�
					NowFrame = Movie->PF.MFTotalFrame - 1 ;
				}
			}

			// ��]�̃t���[�������݃o�b�t�@�Ɋi�[����Ă���t���[���ƈႤ�ꍇ�̓o�b�t�@���X�V����
			if( Movie->PF.MFCurrentFrame != NowFrame )
			{
				Movie->NowImageUpdateFlag = TRUE ;

				// ���Z����t���[�������Z�o
				if( NowFrame < Movie->PF.MFCurrentFrame )
				{
					// ���[�v����ꍇ
					AddFrame = Movie->PF.MFTotalFrame - Movie->PF.MFCurrentFrame + NowFrame ;

					// �Đ��ʒu��ύX
					SeekMovieToFrame_PF( Movie, NowFrame ) ;

#ifndef DX_NON_SOUND
					// ���[�v����ꍇ�ŁA�Đ��^�C�v�������̏ꍇ�̓T�E���h���ēx�Đ����J�n����
					if( Movie->PF.MFLoopType == 0 )
					{
						NS_PlaySoundMem( Movie->PF.MFSoundHandle, DX_PLAYTYPE_BACK ) ;
					}
#endif // DX_NON_SOUND
				}
				else
				{
					AddFrame = NowFrame - Movie->PF.MFCurrentFrame ;

					// �ŏI�X�V���Ԃ��� 2�t���[�����ȏ㎞�Ԃ��o�߂��Ă��Ȃ��̂� 2�t���[���i�ނ��ƂɂȂ����ꍇ�� 1�t���[���ɕ␳����
					if( AddFrame == 2 && ( Movie->RefreshTime - NowTime + ( Movie->PF.FrameTime / 10 ) / 10 ) / ( Movie->PF.FrameTime / 10 ) < 2 )
					{
						AddFrame = 1 ;
					}

					// �t���[����i�߂�
					MediaFoundationMovie_IncToFrame( Movie, AddFrame ) ;
				}

				// �ŏI�X�V���Ԃ�ۑ�
				Movie->RefreshTime = NowTime ;
			}
			else
			{
				// ��]�̃t���[�������t���[�����I�[�ɒB���Ă��Ċ����[�v�w��ł������T�E���h�������ꍇ�͂����ōĐ��I��
				if( NowFrame >= Movie->PF.MFTotalFrame - 1 &&
					( Movie->PlayType & DX_PLAYTYPE_LOOPBIT ) == 0
#ifndef DX_NON_SOUND
					&& ( Movie->PF.MFSoundHandle == -1 || NS_CheckSoundMem( Movie->PF.MFSoundHandle ) != 1 )
#endif // DX_NON_SOUND
					)
				{
					Movie->PlayFlag = FALSE ;

					// �����ꎞ��~�t���O�𗧂Ă�
					Movie->SysPauseFlag = 1 ;
				}
			}
		}
		else
		{
#ifndef DX_NON_FILTER
			if( GSYS.HardInfo.UseShader )
			{
				if( Movie->PF.MFYUVGrHandleSetup == 0 )
				{
					MediaFoundationMovie_SetupImage( Movie, 0, 1, FALSE ) ;
				}
			}
			else
#endif // DX_NON_FILTER
			{
				if( Movie->PF.MFBaseImageSetup == 0 )
				{
					MediaFoundationMovie_SetupImage( Movie, 1, 0, FALSE ) ;
				}
			}
		}

		// �C���[�W�̍\�z
		if( Movie->UpdateFunction )
		{
			Movie->UpdateFunction( Movie, Movie->UpdateFunctionData ) ;

			// �ŏ��̍X�V���s��ꂽ�t���O���Z�b�g����
			Movie->FirstUpdateFlag = TRUE ;
		}
	}
#endif // DX_NON_MEDIA_FOUNDATION

#ifndef DX_NON_DSHOW_MOVIE
	if( Movie->PF.pGraph != NULL )
	{
		LONGLONG Now, Stop ;

		if( Movie->PF.pMovieImage->NewImageSet )
		{
			Movie->NowImageUpdateFlag = TRUE ;

			Movie->PF.pMovieImage->NewImageSet = 0 ;
			if( Movie->UpdateFunction )
			{
				if( Movie->PF.pMovieImage->ImageBuffer &&
					( Movie->PF.pMovieImage->YImageBuffer == NULL || Movie->PF.pMovieImage->AlwaysBaseImage ) )
				{
					Movie->NowImage.Width        = ( int )Movie->PF.pMovieImage->Width ;
					Movie->NowImage.Height       = ( int )Movie->PF.pMovieImage->Height ;
					Movie->NowImage.Pitch        = ( int )Movie->PF.pMovieImage->Pitch ;
					Movie->NowImage.GraphData    = Movie->PF.pMovieImage->ImageBuffer ;
					Movie->NowImageGraphOutAlloc = TRUE ;
					if( Movie->PF.pMovieImage->ImageType == 0 )
					{
						NS_CreateFullColorData( &Movie->NowImage.ColorData ) ;
					}
					else
					if( Movie->PF.pMovieImage->ImageType == 1 && Movie->A8R8G8B8Flag )
					{
						NS_CreateARGB8ColorData( &Movie->NowImage.ColorData ) ;
					}
					else
					{
						NS_CreateXRGB8ColorData( &Movie->NowImage.ColorData ) ;
					}
					Movie->UseNowImage = &Movie->NowImage ;
					Movie->UpdateFunction( Movie, Movie->UpdateFunctionData ) ;
				}

				if( Movie->PF.pMovieImage->YImageBuffer )
				{
#ifndef DX_NON_FILTER
					SETUP_GRAPHHANDLE_GPARAM GParam ;
					BASEIMAGE BaseImage ;
					RECT SrcRect ;

					// �O���t�B�b�N�n���h�����܂��쐬����Ă��Ȃ�������쐬����
					if( Movie->YGrHandle == -1 )
					{
						Graphics_Image_InitSetupGraphHandleGParam( &GParam ) ;

						Graphics_Image_InitSetupGraphHandleGParam_Normal_NonDrawValid( &GParam, 32, FALSE ) ;
						GParam.CreateImageChannelNum      = 1 ;
						GParam.CreateImageChannelBitDepth = 8 ;
						Movie->YGrHandle = Graphics_Image_MakeGraph_UseGParam( &GParam, Movie->PF.pMovieImage->YWidth, Movie->PF.pMovieImage->YHeight, FALSE, FALSE, 0, FALSE, FALSE ) ;
						NS_SetDeleteHandleFlag( Movie->YGrHandle, ( int * )&Movie->YGrHandle ) ;
					}
					if( Movie->UVGrHandle == -1 )
					{
						Graphics_Image_InitSetupGraphHandleGParam( &GParam ) ;

						Graphics_Image_InitSetupGraphHandleGParam_Normal_NonDrawValid( &GParam, 32, FALSE ) ;
						GParam.CreateImageChannelNum      = 2 ;
						GParam.CreateImageChannelBitDepth = 8 ;
						Movie->UVGrHandle     = Graphics_Image_MakeGraph_UseGParam( &GParam, Movie->PF.pMovieImage->UVWidth, Movie->PF.pMovieImage->UVHeight, FALSE, FALSE, 0, FALSE, FALSE ) ;
						NS_SetDeleteHandleFlag( Movie->UVGrHandle, ( int * )&Movie->UVGrHandle ) ;
					}

					_MEMSET( &BaseImage, 0, sizeof( BaseImage ) ) ;
					BaseImage.Width                     = Movie->PF.pMovieImage->YWidth ;
					BaseImage.Height                    = Movie->PF.pMovieImage->YHeight ;
					BaseImage.Pitch                     = Movie->PF.pMovieImage->YPitch ;
					BaseImage.GraphData                 = Movie->PF.pMovieImage->YImageBuffer ;
					BaseImage.ColorData.Format          = DX_BASEIMAGE_FORMAT_NORMAL ;
					BaseImage.ColorData.PixelByte       = 1 ;
					BaseImage.ColorData.ChannelNum      = 1 ;
					BaseImage.ColorData.ChannelBitDepth = 8 ;
					SETRECT( SrcRect, 0, 0, BaseImage.Width, BaseImage.Height ) ;
					Graphics_Image_BltBmpOrBaseImageToGraph3(
						&SrcRect,
						0,
						0,
						Movie->YGrHandle,
						&BaseImage,
						NULL,
						FALSE,
						FALSE,
						FALSE,
						FALSE
					) ;

					BaseImage.Width                     = Movie->PF.pMovieImage->UVWidth ;
					BaseImage.Height                    = Movie->PF.pMovieImage->UVHeight ;
					BaseImage.Pitch                     = Movie->PF.pMovieImage->UVPitch ;
					BaseImage.GraphData                 = Movie->PF.pMovieImage->UVImageBuffer ;
					BaseImage.ColorData.Format          = DX_BASEIMAGE_FORMAT_NORMAL ;
					BaseImage.ColorData.PixelByte       = 2 ;
					BaseImage.ColorData.ChannelNum      = 2 ;
					BaseImage.ColorData.ChannelBitDepth = 8 ;
					SETRECT( SrcRect, 0, 0, BaseImage.Width, BaseImage.Height ) ;
					Graphics_Image_BltBmpOrBaseImageToGraph3(
						&SrcRect,
						0,
						0,
						Movie->UVGrHandle,
						&BaseImage,
						NULL,
						FALSE,
						FALSE,
						FALSE,
						FALSE
					) ;
#endif // DX_NON_FILTER

					Movie->UpdateFunction( Movie, Movie->UpdateFunctionData ) ;
				}
			}
		}

		if( Movie->PF.pMediaSeeking && Movie->PF.pMediaControl )
		{
			Movie->PF.pMediaSeeking->GetCurrentPosition( &Now ) ;
			if( Now >= Movie->StopTime )
			{
				if( Movie->PlayType & DX_PLAYTYPE_LOOPBIT )
				{
					Now = 0 ;
					Stop = 0 ;
					Movie->PF.pMediaSeeking->SetPositions( &Now, D_AM_SEEKING_AbsolutePositioning, &Stop, D_AM_SEEKING_NoPositioning ) ;
					Movie->PF.pMediaControl->Run() ;
				}
				else
				{
					// ��~
					Movie->PF.pMediaControl->Pause() ;

					Movie->PlayFlag = FALSE ;

					// �����ꎞ��~�t���O�𗧂Ă�
					Movie->SysPauseFlag = 1 ;
				}
			}
		}
	}
#endif // DX_NON_DSHOW_MOVIE

	// ����I��
	return 0 ;
}





#ifndef DX_NON_NAMESPACE

}

#endif // DX_NON_NAMESPACE

#endif // DX_NON_MOVIE

