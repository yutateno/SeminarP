#include "../VertexLighting_PS.h"

// main�֐�
PS_OUTPUT main( PS_INPUT PSInput )
{
	PS_OUTPUT PSOutput ;
	float4 TextureDiffuseColor ;
	float4 TextureSpecularColor ;
	float3 Normal ;
	float4 TotalDiffuse ;
	float4 TotalSpecular ;
	float TotalAngleGen ;
	float DiffuseAngleGen ;
	float3 TempF3 ;
	float Temp ;
	float3 ShadowRate ;
	float ShadowGen ;
	float ParamAlpha;

#if SHADOWMAP
	float2 DepthTexCoord ;
	float4 TextureDepth ;
#endif

	ParamAlpha = PSInput.Diffuse.a ;

	TextureDiffuseColor = max( tex2D( DiffuseMapTexture, PSInput.TexCoords0_1.xy ), cfIgnoreTextureColor ) ;

// #ifdef IGNORE_COLOR
// 	TextureDiffuseColor.rgb = 1.0f;
// #endif
// 
// #ifdef IGNORE_TEXALPHA
// 	TextureDiffuseColor.a = 1.0f ;
// #endif

#ifdef INVERSE_COLOR
	TextureDiffuseColor.rgb = 1.0f - TextureDiffuseColor.rgb;
#endif
	
#ifdef USE_SPE
	#ifdef USE_SPETEX
	TextureSpecularColor = tex2D( SpecularMapTexture, PSInput.TexCoords0_1.xy ) ;
#ifdef INVERSE_COLOR
	TextureSpecularColor.rgb = 1.0f - TextureSpecularColor.rgb;
#endif
	#endif // USE_SPETEX
#endif // USE_SPE

	// ���C�g���g��Ȃ��ꍇ�̓J���[�����̂܂܏o�� *****************************************( �J�n )
	#ifndef LG_USE

		// �V���h�E�}�b�v�ւ̕`�� *********************************************************( �J�n )
		#if SHADOWMAP_DRAW
			// �o�̓� = �e�N�X�`���� * �f�B�t���[�Y�� * ��惿
			PSOutput.Color0.a = TextureDiffuseColor.a * ParamAlpha ;
			
			// �y�l��F�Ƃ��ďo��
			PSOutput.Color0.r = PSInput.ShadowMap1Pos_ShadowMap3PosX.z;
			PSOutput.Color0.g = 0.0f ;
			PSOutput.Color0.b = 0.0f ;
		
		// �V���h�E�}�b�v�ւ̕`�� *********************************************************( �I�� )
		#else // SHADOWMAP_DRAW
		
			#ifdef USE_SPE

				#ifdef USE_SPETEX

					PSOutput.Color0 = PSInput.Specular * TextureSpecularColor + PSInput.Diffuse * TextureDiffuseColor ;

				#else  // USE_SPETEX

					PSOutput.Color0 = PSInput.Specular + PSInput.Diffuse * TextureDiffuseColor ;

				#endif // USE_SPETEX

			#else  // USE_SPE

				PSOutput.Color0 = PSInput.Diffuse * TextureDiffuseColor ;

			#endif // USE_SPE

			PSOutput.Color0.a = TextureDiffuseColor.a * ParamAlpha ;
			
		#endif // SHADOWMAP_DRAW

	// ���C�g���g��Ȃ��ꍇ�̓J���[�����̂܂܏o�� *****************************************( �I�� )
	#else // LG_USE
	// ���C�g���g���ꍇ *******************************************************************( �J�n )

		// �o���v�}�b�v or �t�H���V�F�[�f�B���O�̏ꍇ =========================================( �J�n )
		#if BUMPMAP || PHONG

			#if BUMPMAP

				// �@���� 0�`1 �̒l�� -1.0�`1.0 �ɕϊ�����
				Normal = ( tex2D( NormalMapTexture, PSInput.TexCoords0_1.xy ).rgb - cfZeroHalfOneTwo.y ) * cfZeroHalfOneTwo.w ;

			#else // BUMPMAP

				Normal = PSInput.Normal_Fog.xyz ;

			#endif // BUMPMAP


			#ifdef    PHONG
				Normal = normalize( Normal ) ;
			#endif

			#if	SHADOWMAP
				// ���_�̃e�N�X�`�����W�l���͈͓��̏ꍇ�̂ݏ�������
				if( PSInput.ShadowMap1Pos_ShadowMap3PosX.x < -1.0f || PSInput.ShadowMap1Pos_ShadowMap3PosX.x > 1.0f ||
					PSInput.ShadowMap1Pos_ShadowMap3PosX.y < -1.0f || PSInput.ShadowMap1Pos_ShadowMap3PosX.y > 1.0f ||
					PSInput.ShadowMap1Pos_ShadowMap3PosX.z <  0.0f || PSInput.ShadowMap1Pos_ShadowMap3PosX.z > 1.0f )
				{
					ShadowRate.x = 1.0f;
				}
				else
				{
					// �[�x�e�N�X�`���̍��W���Z�o
					// PSInput.ShadowMap1Pos_ShadowMap3PosX.xy �� -1.0f �` 1.0f �̒l�Ȃ̂ŁA����� 0.0f �` 1.0f �̒l�ɂ���
					DepthTexCoord.x = ( PSInput.ShadowMap1Pos_ShadowMap3PosX.x + 1.0f ) / 2.0f;

					// y�͍X�ɏ㉺���]
					DepthTexCoord.y = 1.0f - ( PSInput.ShadowMap1Pos_ShadowMap3PosX.y + 1.0f ) / 2.0f;

					// �[�x�o�b�t�@�e�N�X�`������[�x���擾
					TextureDepth = tex2D( ShadowMap1Texture, DepthTexCoord );

					// �e�N�X�`���ɋL�^����Ă���[�x( +�␳�l )���y�l���傫�������牜�ɂ���Ƃ������ƂŌ��������ő�ɂ���
					ShadowRate.x = smoothstep( PSInput.ShadowMap1Pos_ShadowMap3PosX.z - cfShadowMap1_DAdj_Grad_Enbl0_1.y, PSInput.ShadowMap1Pos_ShadowMap3PosX.z, TextureDepth.r + cfShadowMap1_DAdj_Grad_Enbl0_1.x ) ;
				}

				// ���_�̃e�N�X�`�����W�l���͈͓��̏ꍇ�̂ݏ�������
				if( PSInput.ShadowMap2Pos_ShadowMap3PosY.x < -1.0f || PSInput.ShadowMap2Pos_ShadowMap3PosY.x > 1.0f ||
					PSInput.ShadowMap2Pos_ShadowMap3PosY.y < -1.0f || PSInput.ShadowMap2Pos_ShadowMap3PosY.y > 1.0f ||
					PSInput.ShadowMap2Pos_ShadowMap3PosY.z <  0.0f || PSInput.ShadowMap2Pos_ShadowMap3PosY.z > 1.0f )
				{
					ShadowRate.y = 1.0f;
				}
				else
				{
					// �[�x�e�N�X�`���̍��W���Z�o
					// PSInput.ShadowMap2Pos_ShadowMap3PosY.xy �� -1.0f �` 1.0f �̒l�Ȃ̂ŁA����� 0.0f �` 1.0f �̒l�ɂ���
					DepthTexCoord.x = ( PSInput.ShadowMap2Pos_ShadowMap3PosY.x + 1.0f ) / 2.0f;

					// y�͍X�ɏ㉺���]
					DepthTexCoord.y = 1.0f - ( PSInput.ShadowMap2Pos_ShadowMap3PosY.y + 1.0f ) / 2.0f;

					// �[�x�o�b�t�@�e�N�X�`������[�x���擾
					TextureDepth = tex2D( ShadowMap2Texture, DepthTexCoord );

					// �e�N�X�`���ɋL�^����Ă���[�x( +�␳�l )���y�l���傫�������牜�ɂ���Ƃ������ƂŌ��������ő�ɂ���
					ShadowRate.y = smoothstep( PSInput.ShadowMap2Pos_ShadowMap3PosY.z - cfShadowMap1_Enb2_ShadowMap2_DAdj_Grad_Enbl0.z, PSInput.ShadowMap2Pos_ShadowMap3PosY.z, TextureDepth.r + cfShadowMap1_Enb2_ShadowMap2_DAdj_Grad_Enbl0.y ) ;
				}

				// ���_�̃e�N�X�`�����W�l���͈͓��̏ꍇ�̂ݏ�������
				if( PSInput.ShadowMap1Pos_ShadowMap3PosX.w < -1.0f || PSInput.ShadowMap1Pos_ShadowMap3PosX.w > 1.0f ||
					PSInput.ShadowMap2Pos_ShadowMap3PosY.w < -1.0f || PSInput.ShadowMap2Pos_ShadowMap3PosY.w > 1.0f ||
					PSInput.V_to_Eye_ShadowMap3PosZ.w      <  0.0f || PSInput.V_to_Eye_ShadowMap3PosZ.w      > 1.0f )
				{
					ShadowRate.z = 1.0f;
				}
				else
				{
					// �[�x�e�N�X�`���̍��W���Z�o
					// PSInput.ShadowMap1Pos_ShadowMap3PosX.w �� PSInput.ShadowMap2Pos_ShadowMap3PosY.w �� -1.0f �` 1.0f �̒l�Ȃ̂ŁA����� 0.0f �` 1.0f �̒l�ɂ���
					DepthTexCoord.x = ( PSInput.ShadowMap1Pos_ShadowMap3PosX.w + 1.0f ) / 2.0f;

					// y�͍X�ɏ㉺���]
					DepthTexCoord.y = 1.0f - ( PSInput.ShadowMap2Pos_ShadowMap3PosY.w + 1.0f ) / 2.0f;

					// �[�x�o�b�t�@�e�N�X�`������[�x���擾
					TextureDepth = tex2D( ShadowMap3Texture, DepthTexCoord );

					// �e�N�X�`���ɋL�^����Ă���[�x( +�␳�l )���y�l���傫�������牜�ɂ���Ƃ������ƂŌ��������ő�ɂ���
					ShadowRate.z = smoothstep( PSInput.V_to_Eye_ShadowMap3PosZ.w - cfShadowMap2_Enbl1_2_ShadowMap3_DAdj_Grad.w, PSInput.V_to_Eye_ShadowMap3PosZ.w, TextureDepth.r + cfShadowMap2_Enbl1_2_ShadowMap3_DAdj_Grad.z ) ;
				}

			#else // SHADOWMAP
				ShadowRate.x = 1.0f ;
				ShadowRate.y = 1.0f ;
				ShadowRate.z = 1.0f ;
			#endif // SHADOWMAP

			// �f�B�t���[�Y�J���[�ƃX�y�L�����J���[�̒~�ϒl��������
			TotalDiffuse = cfZeroHalfOneTwo.x ;
			TotalSpecular = cfZeroHalfOneTwo.x ;




			
// ���C�g0�̏��� +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++( �J�n )
#if LG0_USE

	// �f�B�t���[�Y�F�v�Z

	// DiffuseAngleGen = �f�B�t���[�Y�p�x�������v�Z
	DiffuseAngleGen = saturate( dot( Normal, PSInput.Light0_Dir_Gen.xyz ) ) ;
	
	// �e�ɂ�錸�����Z�o
	ShadowGen = max( ShadowRate.x, SHADOWMAP1_ENABLE_LGT0 ) *
	            max( ShadowRate.y, SHADOWMAP2_ENABLE_LGT0 ) *
	            max( ShadowRate.z, SHADOWMAP3_ENABLE_LGT0 ) ;

	#if LG0_DIR

		// �f�B�t���[�Y�J���[�~�ϒl += ���C�g�̃f�B�t���[�Y�J���[ * �f�B�t���[�Y�J���[ * �f�B�t���[�Y�J���[�p�x������ + ���C�g�̃A���r�G���g�J���[�ƃ}�e���A���̃A���r�G���g�J���[����Z�������� 
		TotalDiffuse += cfLight[ 0 ].Diffuse * PSInput.Diffuse * DiffuseAngleGen * ShadowGen + cfLight[ 0 ].Ambient ;

	#else // LG0_DIR

		// �f�B�t���[�Y�J���[�~�ϒl += ( ���C�g�̃f�B�t���[�Y�J���[ * �f�B�t���[�Y�J���[ * �f�B�t���[�Y�J���[�p�x������ + ���C�g�̃A���r�G���g�J���[�ƃ}�e���A���̃A���r�G���g�J���[����Z��������  ) * �����E�X�|�b�g���C�g�̊p�x������
		TotalDiffuse += ( cfLight[ 0 ].Diffuse * PSInput.Diffuse * DiffuseAngleGen * ShadowGen + cfLight[ 0 ].Ambient ) * PSInput.Light0_Dir_Gen.w ;

	#endif // LG0_DIR


	// �X�y�L�����J���[�v�Z
	#ifdef USE_SPE

		// �n�[�t�x�N�g���̌v�Z
		TempF3 = PSInput.V_to_Eye_ShadowMap3PosZ.xyz + PSInput.Light0_Dir_Gen.xyz ;
		#ifdef    PHONG
			TempF3 = normalize( TempF3 ) ;
		#else  // PHONG
			TempF3 *= 0.5f ;
		#endif // PHONG

		// Temp = pow( max( 0.0f, N * H ), cfMaterial.Power.x )
		Temp = pow( max( 0.0f, dot( Normal, TempF3 ) ), cfMaterial.Power.x ) ;

		#if LG0_DIR

			// �X�y�L�����J���[�~�ϒl += Temp * ���C�g�̃X�y�L�����J���[
			TotalSpecular += Temp * cfLight[ 0 ].Specular * ShadowGen ;

		#else // LG0_DIR

			// �X�y�L�����J���[�~�ϒl += Temp * �����E�X�|�b�g���C�g�̊p�x������ * ���C�g�̃X�y�L�����J���[
			TotalSpecular += Temp * PSInput.Light0_Dir_Gen.w * cfLight[ 0 ].Specular * ShadowGen ;

		#endif // LG0_DIR

	#endif // USE_SPE

#endif // LG0_USE
// ���C�g0�̏��� +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++( �I�� )









			
// ���C�g1�̏��� +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++( �J�n )
#if LG1_USE

	// �f�B�t���[�Y�F�v�Z

	// DiffuseAngleGen = �f�B�t���[�Y�p�x�������v�Z
	DiffuseAngleGen = saturate( dot( Normal, PSInput.Light1_Dir_Gen.xyz ) ) ;
	
	// �e�ɂ�錸�����Z�o
	ShadowGen = max( ShadowRate.x, SHADOWMAP1_ENABLE_LGT1 ) *
	            max( ShadowRate.y, SHADOWMAP2_ENABLE_LGT1 ) *
	            max( ShadowRate.z, SHADOWMAP3_ENABLE_LGT1 ) ;

	#if LG1_DIR

		// �f�B�t���[�Y�J���[�~�ϒl += ���C�g�̃f�B�t���[�Y�J���[ * �f�B�t���[�Y�J���[ * �f�B�t���[�Y�J���[�p�x������ + ���C�g�̃A���r�G���g�J���[�ƃ}�e���A���̃A���r�G���g�J���[����Z�������� 
		TotalDiffuse += cfLight[ 1 ].Diffuse * PSInput.Diffuse * DiffuseAngleGen * ShadowGen + cfLight[ 1 ].Ambient ;

	#else // LG1_DIR

		// �f�B�t���[�Y�J���[�~�ϒl += ( ���C�g�̃f�B�t���[�Y�J���[ * �f�B�t���[�Y�J���[ * �f�B�t���[�Y�J���[�p�x������ + ���C�g�̃A���r�G���g�J���[�ƃ}�e���A���̃A���r�G���g�J���[����Z��������  ) * �����E�X�|�b�g���C�g�̊p�x������
		TotalDiffuse += ( cfLight[ 1 ].Diffuse * PSInput.Diffuse * DiffuseAngleGen * ShadowGen + cfLight[ 1 ].Ambient ) * PSInput.Light1_Dir_Gen.w ;

	#endif // LG1_DIR


	// �X�y�L�����J���[�v�Z
	#ifdef USE_SPE

		// �n�[�t�x�N�g���̌v�Z
		TempF3 = PSInput.V_to_Eye_ShadowMap3PosZ.xyz + PSInput.Light1_Dir_Gen.xyz ;
		#ifdef    PHONG
			TempF3 = normalize( TempF3 ) ;
		#else  // PHONG
			TempF3 *= 0.5f ;
		#endif // PHONG

		// Temp = pow( max( 0.0f, N * H ), cfMaterial.Power.x )
		Temp = pow( max( 0.0f, dot( Normal, TempF3 ) ), cfMaterial.Power.x ) ;

		#if LG1_DIR

			// �X�y�L�����J���[�~�ϒl += Temp * ���C�g�̃X�y�L�����J���[
			TotalSpecular += Temp * cfLight[ 1 ].Specular * ShadowGen ;

		#else // LG1_DIR

			// �X�y�L�����J���[�~�ϒl += Temp * �����E�X�|�b�g���C�g�̊p�x������ * ���C�g�̃X�y�L�����J���[
			TotalSpecular += Temp * PSInput.Light1_Dir_Gen.w * cfLight[ 1 ].Specular * ShadowGen ;

		#endif // LG1_DIR

	#endif // USE_SPE

#endif // LG1_USE
// ���C�g1�̏��� +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++( �I�� )









			
// ���C�g2�̏��� +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++( �J�n )
#if LG2_USE

	// �f�B�t���[�Y�F�v�Z

	// DiffuseAngleGen = �f�B�t���[�Y�p�x�������v�Z
	DiffuseAngleGen = saturate( dot( Normal, PSInput.Light2_Dir_Gen.xyz ) ) ;
	
	// �e�ɂ�錸�����Z�o
	ShadowGen = max( ShadowRate.x, SHADOWMAP1_ENABLE_LGT2 ) *
	            max( ShadowRate.y, SHADOWMAP2_ENABLE_LGT2 ) *
	            max( ShadowRate.z, SHADOWMAP3_ENABLE_LGT2 ) ;

	#if LG2_DIR

		// �f�B�t���[�Y�J���[�~�ϒl += ���C�g�̃f�B�t���[�Y�J���[ * �f�B�t���[�Y�J���[ * �f�B�t���[�Y�J���[�p�x������ + ���C�g�̃A���r�G���g�J���[�ƃ}�e���A���̃A���r�G���g�J���[����Z�������� 
		TotalDiffuse += cfLight[ 2 ].Diffuse * PSInput.Diffuse * DiffuseAngleGen * ShadowGen + cfLight[ 2 ].Ambient ;

	#else // LG2_DIR

		// �f�B�t���[�Y�J���[�~�ϒl += ( ���C�g�̃f�B�t���[�Y�J���[ * �f�B�t���[�Y�J���[ * �f�B�t���[�Y�J���[�p�x������ + ���C�g�̃A���r�G���g�J���[�ƃ}�e���A���̃A���r�G���g�J���[����Z��������  ) * �����E�X�|�b�g���C�g�̊p�x������
		TotalDiffuse += ( cfLight[ 2 ].Diffuse * PSInput.Diffuse * DiffuseAngleGen * ShadowGen + cfLight[ 2 ].Ambient ) * PSInput.Light2_Dir_Gen.w ;

	#endif // LG2_DIR


	// �X�y�L�����J���[�v�Z
	#ifdef USE_SPE

		// �n�[�t�x�N�g���̌v�Z
		TempF3 = PSInput.V_to_Eye_ShadowMap3PosZ.xyz + PSInput.Light2_Dir_Gen.xyz ;
		#ifdef    PHONG
			TempF3 = normalize( TempF3 ) ;
		#else  // PHONG
			TempF3 *= 0.5f ;
		#endif // PHONG

		// Temp = pow( max( 0.0f, N * H ), cfMaterial.Power.x )
		Temp = pow( max( 0.0f, dot( Normal, TempF3 ) ), cfMaterial.Power.x ) ;

		#if LG2_DIR

			// �X�y�L�����J���[�~�ϒl += Temp * ���C�g�̃X�y�L�����J���[
			TotalSpecular += Temp * cfLight[ 2 ].Specular * ShadowGen ;

		#else // LG2_DIR

			// �X�y�L�����J���[�~�ϒl += Temp * �����E�X�|�b�g���C�g�̊p�x������ * ���C�g�̃X�y�L�����J���[
			TotalSpecular += Temp * PSInput.Light2_Dir_Gen.w * cfLight[ 2 ].Specular * ShadowGen ;

		#endif // LG2_DIR

	#endif // USE_SPE

#endif // LG2_USE
// ���C�g2�̏��� +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++( �I�� )


















			// �o�̓J���[�v�Z +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++( �J�n )

			// TotalDiffuse = ���C�g�f�B�t���[�Y�J���[�~�ϒl + ( �}�e���A���̃A���r�G���g�J���[�ƃO���[�o���A���r�G���g�J���[����Z�������̂ƃ}�e���A���G�~�b�V�u�J���[�����Z�������� )
			TotalDiffuse += cfAmbient_Emissive ;

			#ifdef USE_SPE
				#ifdef USE_SPETEX
					// TextureSpecularColor = ���C�g�̃X�y�L�����J���[�~�ϒl * �X�y�L�����J���[ * �X�y�L�����e�N�X�`���J���[
					TextureSpecularColor = tex2D( SpecularMapTexture, PSInput.TexCoords0_1.xy ) * TotalSpecular * PSInput.Specular ;
				#else  // USE_SPETEX
					// TextureSpecularColor = ���C�g�̃X�y�L�����J���[�~�ϒl * �X�y�L�����J���[
					TextureSpecularColor = TotalSpecular * PSInput.Specular ;
				#endif // USE_SPETEX

				// �o�̓J���[ = TotalDiffuse * �e�N�X�`���J���[ + TextureSpecularColor
				PSOutput.Color0.rgb = TextureDiffuseColor.rgb * TotalDiffuse.rgb + TextureSpecularColor.rgb ;

				// �o�̓� = �e�N�X�`���� * �f�B�t���[�Y�� * ��惿
				PSOutput.Color0.a = TextureDiffuseColor.a * ParamAlpha ;

			#else  // USE_SPE

				// �o�̓J���[ = TotalDiffuse * �e�N�X�`���J���[
				PSOutput.Color0.rgb = TotalDiffuse.rgb * TextureDiffuseColor.rgb ;

				// �o�̓� = �e�N�X�`���� * �f�B�t���[�Y�� * ��惿
				PSOutput.Color0.a = TextureDiffuseColor.a * ParamAlpha ;

			#endif // USE_SPE

			// �o�̓J���[�v�Z +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++( �I�� )


		// �o���v�}�b�v or �t�H���V�F�[�f�B���O�̏ꍇ =========================================( �I�� )
		#else  // BUMPMAP || PHONG
			// �o���v�}�b�v or �t�H���V�F�[�f�B���O�ł͂Ȃ��ꍇ ===================================( �J�n )

			#ifdef USE_SPE

				#ifdef USE_SPETEX

					PSOutput.Color0.rgb = TextureDiffuseColor.rgb * PSInput.Diffuse.rgb + TextureSpecularColor.rgb * PSInput.Specular.rgb ;

				#else  // USE_SPETEX

					PSOutput.Color0.rgb = TextureDiffuseColor.rgb * PSInput.Diffuse.rgb + PSInput.Specular.rgb ;

				#endif // USE_SPETEX

			#else  // USE_SPE

				PSOutput.Color0.rgb = TextureDiffuseColor.rgb * PSInput.Diffuse.rgb ;

			#endif // USE_SPE

			PSOutput.Color0.a   = TextureDiffuseColor.a   * ParamAlpha ;

		#endif // BUMPMAP || PHONG
		// �o���v�}�b�v or �t�H���V�F�[�f�B���O�ł͂Ȃ��ꍇ ===================================( �I�� )

	#endif // LG_USE
	// ���C�g���g���ꍇ *******************************************************************( �I�� )

#ifdef PRE_MUL_ALPHA
	PSOutput.Color0.rgb *= PSInput.Diffuse.a ;
#endif

#ifdef MUL_X_4
	PSOutput.Color0.rgb *= 4.0f ;
#endif

#ifdef MUL_BLEND
	PSOutput.Color0 = lerp( 1.0f, PSOutput.Color0, PSOutput.Color0.a ) ;
#endif

#if SM_3
	// �t�H�O����
	PSOutput.Color0.rgb = lerp( cfFogColor.rgb, PSOutput.Color0.rgb, saturate( PSInput.Normal_Fog.w ) );
#endif // SM_3

	return PSOutput ;
}

