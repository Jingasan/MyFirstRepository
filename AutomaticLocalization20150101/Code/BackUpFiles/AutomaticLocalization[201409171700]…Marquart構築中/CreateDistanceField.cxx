/***** �f�B�X�^���X�t�B�[���h�̍쐬 *****/


/*** �C���N���[�h�t�@�C�� ***/

/* �ݒ�p�w�b�_�t�@�C�� */
#include "Configuration.h"
#include "FunctionDefinition.h"


/*** ���O��Ԃ̐錾 ***/

/* C++ */
using namespace std;


/*** �T���Ώۓ_�Q�ɑ΂��ăf�B�X�^���X�t�B�[���h���쐬 ***/
void CreateDistanceFieldforPCDModel(
	int DistanceFieldDevideSize,				// DF �̕�����( DF �̃{�N�Z���̕���\ )
	int** DistanceField,						// DF
	int** ClosestPointID,						// �ŋߓ_�� ID
	int* ModelPointCloudSize,					// �T���Ώۓ_�Q��
	struct ModelPointCloud** modelpointcloud,	// �T���Ώۓ_�Q( DF �̍��W�n�ɒ���������, �P��:mm )
	struct ModelInformation* modelinformation,	// �T���Ώۓ_�Q�̊e����
	pcl::PointCloud< pcl::PointXYZ >::Ptr cloud	// �T���Ώۓ_�Q( �P��:m )
){
	
	/* �ϐ��̒�` */
	int i, j, k, l;				// DF �� ID �� i, j, k
    int dist;					// �{�N�Z���ƃp�b�`�Ƃ̋���
    int *DFtemp;				// DF �쐬�̂��߂Ɉꎞ�I�ɕۑ����锠
    int *CPtemp;				// CP �쐬�̂��߂Ɉꎞ�I�ɕۑ����锠
    int ID;						// DF �̃C���f�b�N�X
    int *stack;					// DF �쐬�Ɏg�p
    int top;					// stack �� top ���w��
    int infiniFlag;				// ��O�����̃t���O
    int tempi;					// �œK�� i
    int flag;					// ��O�������邩�ǂ���
    int size2;					// size2 = iSize * jSize;
    int size3;					// size3 = iSize * jSize * iSize;
    double *x;					// ��������̌�_��ۑ�
    int iSize, jSize, kSize;	// DF �̃T�C�Y 
	double gravX, gravY, gravZ;	// �_�Q�̒��S���W�����f�����W�n�̌��_�Ƃ���
	
	/* ������ */
	modelinformation->Xmin = INFINITY;
	modelinformation->Ymin = INFINITY;
	modelinformation->Zmin = INFINITY;
	modelinformation->Xmax = -INFINITY;
	modelinformation->Ymax = -INFINITY;
	modelinformation->Zmax = -INFINITY;
	gravX = 0;
	gravY = 0;
	gravZ = 0;
	
	/* �_�Q�̒[�_�̎Z�o */
	for( size_t i = 0; i < cloud->points.size(); i++ ){
		
		/* X, Y, Z �̍ŏ��l��ݒ� */
		if( cloud->points[ i ].x * TOMETER < modelinformation->Xmin ) modelinformation->Xmin = cloud->points[ i ].x * TOMETER;
		if( cloud->points[ i ].y * TOMETER < modelinformation->Ymin ) modelinformation->Ymin = cloud->points[ i ].y * TOMETER;
		if( cloud->points[ i ].z * TOMETER < modelinformation->Zmin ) modelinformation->Zmin = cloud->points[ i ].z * TOMETER;
		
		/* X, Y, Z �̍ő�l��ݒ� */
		if( cloud->points[ i ].x * TOMETER > modelinformation->Xmax ) modelinformation->Xmax = cloud->points[ i ].x * TOMETER;
		if( cloud->points[ i ].y * TOMETER > modelinformation->Ymax ) modelinformation->Ymax = cloud->points[ i ].y * TOMETER;
		if( cloud->points[ i ].z * TOMETER > modelinformation->Zmax ) modelinformation->Zmax = cloud->points[ i ].z * TOMETER;
	}
	
	/* �_�Q�̏d�S�̎Z�o */ 
	gravX = ( modelinformation->Xmax + modelinformation->Xmin ) / 2;
	gravY = ( modelinformation->Ymax + modelinformation->Ymin ) / 2;
	gravZ = ( modelinformation->Zmax + modelinformation->Zmin ) / 2;
	
	/* �_�Q�̏d�S�����f�����Ƃ��Ċi�[ */
	modelinformation->gravX = gravX;
	modelinformation->gravY = gravY;
	modelinformation->gravZ = gravZ;

	/* �_�Q�̏d�S�����_�ɐݒ� */
	modelinformation->Xmin -= gravX; modelinformation->Xmax -= gravX;
	modelinformation->Ymin -= gravY; modelinformation->Ymax -= gravY;
	modelinformation->Zmin -= gravZ; modelinformation->Zmax -= gravZ;
	
	/* �_�Q�̍ő啝�̎Z�o */
	modelinformation->deltaX = modelinformation->Xmax - modelinformation->Xmin;
	modelinformation->deltaY = modelinformation->Ymax - modelinformation->Ymin;
	modelinformation->deltaZ = modelinformation->Zmax - modelinformation->Zmin;
    
	/* �ł��傫���l�����_�Q���̎Z�o X���� or Y���� or Z���� */
	if( modelinformation->deltaX > modelinformation->deltaY ){
		if( modelinformation->deltaX > modelinformation->deltaZ ){
			modelinformation->max = modelinformation->deltaX;
		}else{
			modelinformation->max = modelinformation->deltaZ;
		}
	}else{
		if( modelinformation->deltaY > modelinformation->deltaZ ){
			modelinformation->max = modelinformation->deltaY;
		}else{
			modelinformation->max = modelinformation->deltaZ;
		}
	}

	/* �f�B�X�^���X�t�B�[���h�̃T�C�Y�̎Z�o */
	modelinformation->iSize = (int)( ( modelinformation->deltaX / modelinformation->max ) * DistanceFieldDevideSize );
	modelinformation->jSize = (int)( ( modelinformation->deltaY / modelinformation->max ) * DistanceFieldDevideSize );
	modelinformation->kSize = (int)( ( modelinformation->deltaZ / modelinformation->max ) * DistanceFieldDevideSize );
	iSize = modelinformation->iSize;	// DF �� i �����̃T�C�Y
	jSize = modelinformation->jSize;	// DF �� j �����̃T�C�Y
	kSize = modelinformation->kSize;	// DF �� k �����̃T�C�Y
	size2 = iSize * jSize;				// DF �̖ʐ�
	size3 = iSize * jSize * kSize;		// DF �̑̐�
    
	/* �������̊m�ۂƏ����� */ 
	( *DistanceField ) = new int[ size3 ];	// DF �̃������m��
	( *ClosestPointID ) = new int[ size3 ];	// �ŋߓ_�̃������m��
	DFtemp = new int[ size3 ];
	CPtemp = new int[ size3 ];
	for( i = 0; i < size3; i++ ){
		DFtemp[ i ] = INFINITY;
		( *ClosestPointID )[ i ] = INFINITY;
	}
	stack = new int[ DistanceFieldDevideSize ];
	x = new double[ DistanceFieldDevideSize ];
    


	/*** �f�B�X�^���X�t�B�[���h�̓�l�� ***/
	cout << "--- �y BinarizationDFforPCDModel�֐� �zDF���l�� ---" << endl;
	BinarizationDFforPCDModel(
		DistanceFieldDevideSize,	// DF �̕�����( DF �̃{�N�Z���̕���\ )
		DFtemp,						// DF �쐬�̂��߂Ɉꎞ�I�ɕۑ����锠
		ClosestPointID,				// �ŋߓ_�� ID
		modelpointcloud,			// 3�����_�Q
		ModelPointCloudSize,		// 3�����_�Q�̐�
		modelinformation,			// 3�����_�Q�̊e����
		cloud,						// ����3�����_�Q
		gravX,						// 3�����_�Q�� X �����̏d�S
		gravY,						// 3�����_�Q�� Y �����̏d�S
		gravZ						// 3�����_�Q�� Z �����̏d�S
	);	// �����Ń{�N�Z���ƃp�b�`�Ƃ̋����� 0.5 �ȓ��̂Ƃ��ɋ����� 0 �Ƃ��� DF �ɕۑ�����



    /*** �f�B�X�^���X�t�B�[���h�̍쐬 ***/

	/* Step1�b�c�����ɋ������v�Z���� */
    for( k = 0; k < kSize; k++ ){
		for( i = 0; i < iSize; i++ ){
			dist = INFINITY;
			ID = INFINITY;
			for( j = 0; j < jSize; j++ ){
				if( dist != INFINITY ){
					dist++;
				}
				/* ������0�̓_���������狗��0��ݒ�C���̍ŋߓ_ID��o�^ */
				if( DFtemp[ k * size2 + j * iSize + i ] == 0 ){
					dist = 0;
					ID = ( *ClosestPointID )[ k * size2 + j * iSize + i ];
				}
				( *DistanceField )[ k * size2 + j * iSize + i ] = dist;
				( *ClosestPointID )[ k * size2 + j * iSize + i ] = ID;
			} // for j

			dist = INFINITY;
			ID = INFINITY;
			for( j = jSize - 1; j >= 0; j-- ){
				if( dist != INFINITY ){
					dist++;
				}
				if( DFtemp[ k * size2 + j * iSize + i ] == 0 ){
					dist = 0;
					ID = ( *ClosestPointID )[ k * size2 + j * iSize + i ];
				}
				if( dist < ( *DistanceField )[ k * size2 + j * iSize + i ] ){
					( *DistanceField )[ k * size2 + j * iSize + i ] = dist;
					( *ClosestPointID )[ k * size2 + j * iSize + i ] = ID;
				}
			} // for j
		} // for i
	} // for k
	
	/* Step2�b�������ɋ������v�Z���� */
	for( k = 0; k < kSize; k++ ){
		for( j = 0; j < jSize; j++ ){
			top = 0;
			infiniFlag = 0;
			
			for( l = 0; l < iSize - 1; l++ ){
				if( ( *DistanceField )[ k * size2 + j * iSize + l ] != INFINITY && ( *DistanceField )[ k * size2 + j * iSize + l + 1 ] != INFINITY ){
					x[ top ] = (double)( 1 + 2 * l + ( ( *DistanceField )[ k * size2 + j * iSize + l + 1 ] * ( *DistanceField )[ k * size2 + j * iSize + l + 1 ] ) - ( ( *DistanceField ) [ k * size2 + j * iSize + l ] * ( *DistanceField )[ k * size2 + j * iSize + l ] ) ) / 2;
					stack[ top ] = l;
					top++;
					stack[ top ] = l + 1;
					break;
				} // if
				if( l == ( iSize - 2 ) ){
					infiniFlag = 1;
				}
			} // for l

			if( infiniFlag == 0 ){
				for( i = l + 2; i < iSize; i++ ){
					x[ top ] = (double)( ( stack[ top ] * stack[ top ]) - ( i * i ) + ( ( *DistanceField )[ k * size2 + j * iSize + stack[ top ] ] * ( *DistanceField )[ k * size2 + j * iSize + stack[ top ] ] ) - ( ( *DistanceField )[ k * size2 + j * iSize + i ] * ( *DistanceField )[ k * size2 + j * iSize + i ] ) ) / ( 2 * ( stack[ top ] - i ) );
					if( ( *DistanceField )[ k * size2 + j * iSize + i ] != INFINITY ){
						while( x[ top ] < x[ top - 1 ] && top > 0 ){
							top--;
							x[ top ] = (double)( ( stack[ top ] * stack[ top ] ) - ( i * i ) + ( ( *DistanceField )[ k * size2 + j * iSize + stack[ top ] ] * ( *DistanceField )[ k * size2 + j * iSize + stack[ top ] ] ) - ( ( *DistanceField )[ k * size2 + j * iSize + i ] * ( *DistanceField )[ k * size2 + j * iSize + i ] ) ) / ( 2 * ( stack[ top ] - i ) );
						}
						top++;
						stack[ top ] = i;
					}
				}//for i

				while( (double)( iSize - 1 ) < x[ top - 1 ] ){
					top--;
				}
				
				for( i = iSize - 1; i >= 0; i-- ){
					while( top > 0 && (double)i <= x[ top - 1 ] ){
						top--;
						if( top == 0 ) break;
					}
					DFtemp[ k * size2 + j * iSize + i ] = ( ( i - stack[ top ] ) * ( i - stack[ top ] ) ) + ( ( *DistanceField )[ k * size2 + j * iSize + stack[ top ] ] * ( *DistanceField )[ k * size2 + j * iSize + stack[ top ] ] );
					CPtemp[ k * size2 + j * iSize + i ] = ( *ClosestPointID )[ k * size2 + j * iSize + stack[ top ] ];
				}
			}else{
				flag = 0;
				for( i = 0; i < iSize; i++ ){
					if( ( *DistanceField )[ k * size2 + j * iSize + i ] != INFINITY ){
						flag = 1;
						tempi = i;
						
						for( l = 0; l < iSize; l++ ){
							dist = ( l - tempi ) * ( l - tempi ) + ( *DistanceField )[ k * size2 + j * iSize + tempi ] * ( *DistanceField )[ k * size2 + j * iSize + tempi ];
							if( DFtemp[ k * size2 + j * iSize + l ] >= dist ){
								DFtemp[ k * size2 + j * iSize + l ] = dist;
								CPtemp[ k * size2 + j * iSize + l ] = ( *ClosestPointID )[ k * size2 + j * iSize + tempi ];
							}
						} // for l
					}else{
						if( flag == 0 ){
							DFtemp[ k * size2 + j * iSize + i ] = INFINITY;
						} // if
					}
				}
			}
		}
	}

	/* Step3�b�������ɋ������v�Z���� */
	for( j = 0; j < jSize; j++ ){
		for( i = 0; i < iSize; i++ ){
			top = 0;
			infiniFlag = 0;
			for( l = 0; l < kSize - 1; l++ ){
				if( DFtemp[ l * size2 + j * iSize + i ] != INFINITY && DFtemp[ l * size2 + j * iSize + i + 1 ] != INFINITY ){
					x[ top ] = (double)( 1 + 2 * l + DFtemp[ ( l + 1 ) * size2 + j * iSize + i ] - DFtemp[ l * size2 + j * iSize + i ] ) / 2;
					stack[ top ] = l;
					top++;
					stack[ top ] = l + 1;
					break;
				}
				if( l == ( kSize - 2 ) ){
					infiniFlag = 1;
				} // if
			}

			if( infiniFlag == 0 ){
				for( k = l + 2; k < kSize; k++ ){
					x[ top ] = (double)( ( stack[ top ] * stack[ top ] ) - ( k * k ) + DFtemp[ stack[ top ] * size2 + j * iSize + i ] - DFtemp[ k * size2 + j * iSize + i ] ) / ( 2 * ( stack[ top ] - k ) );
					if( DFtemp[ k * size2 + j * iSize + i ] != INFINITY ){
						while( x[ top ] < x[ top - 1 ] && top > 0 ){
							top--;
							x[ top ] = (double)( ( stack[ top ] * stack[ top ] ) - ( k * k ) + DFtemp[ stack[ top ] * size2 + j * iSize + i ] - DFtemp[ k * size2 + j * iSize + i ] ) / ( 2 * ( stack[ top ] - k ) );
						}
						top++;
						stack[ top ] = k;
					}
				}

				while( (double)( kSize - 1 ) < x[ top - 1 ] ){
					top--;
				}

				for( k = kSize - 1; k >= 0; k-- ){
					while( top > 0 && (double)k <= x[ top - 1 ] ){
						top--;
						if( top == 0 ){
							break;
						}
					}
					( *DistanceField )[ k * size2 + j * iSize + i ] = ( ( k - stack[ top ] ) * ( k - stack[ top ] ) ) + DFtemp[ stack[ top ] * size2 + j * iSize + i ];
					( *ClosestPointID )[ k * size2 + j * iSize + i ] = CPtemp[ stack[ top ] * size2 + j * iSize + i ];
				}
			}else{
				flag = 0;
				for( k = kSize - 1; k >= 0; k-- ){
					if( ( *DistanceField )[ k * size2 + j * iSize + i ] != INFINITY ){
						flag = 1;
						tempi = k;
						for( l = 0; l < kSize; l++ ){
							dist = ( l - tempi ) * ( l - tempi ) + DFtemp[ tempi * size2 + j * iSize + i ] * DFtemp[ tempi * size2 + j * iSize + i ];
							if( ( *DistanceField )[ l * size2 + j * iSize + i ] >= dist ){
								( *DistanceField )[ l * size2 + j * iSize + i ] = dist;
								( *ClosestPointID )[ l * size2 + j * iSize + i ] = CPtemp[ tempi * size2 + j * iSize + i ];
							}
						}
					}else{
						if( flag == 0 ){
							( *DistanceField )[ k * size2 + j * iSize + i ] = INFINITY;
						}
					}
				} // for k
			}
		} // for i
	} // for j


	/*** �f�B�X�^���X�t�B�[���h�ƍŋߓ_�̏o�� ***/
	cout << "�E�E�E�f�B�X�^���X�t�B�[���h�E�ŋߓ_���o�́E�E�E" << endl;
	FILE *FPDF, *FPCP, *FPDFCP;
	FPDF = fopen( "Output/DistanceField/DistanceField.csv", "w" );
	FPCP = fopen( "Output/DistanceField/ClosestPoint.csv", "w" );
	FPDFCP = fopen("Output/DistanceField/DFandCP.csv", "w" );
	fprintf( FPDF, "�ް��`��,3,\nmemo1,\nX,Y,Z,Distance\n" );
	fprintf( FPCP, "�ް��`��,3,\nmemo1,\nX,Y,Z,ClosestPoint\n" );
	fprintf( FPDFCP, "�ް��`��,3,\nmemo1,\nX,Y,Z,DistanceField,ClosestPoint\n" );
	
	/* �f�B�X�^���X�t�B�[���h�ƍŋߓ_�̏������� */
	for( k = 0; k < kSize; k++ ){
		for( j = 0; j < jSize; j++ ){
			for( i = 0; i < iSize; i++ ){
				fprintf( FPDF, "%d,%d,%d,%d,\n", i, j, k, ( *DistanceField )[ k * size2 + j * iSize + i ] );
				fprintf( FPCP, "%d,%d,%d,%d,\n", i, j, k, ( *ClosestPointID )[ k * size2 + j * iSize + i ] );
				fprintf( FPDFCP,"%d,%d,%d,%d,%d\n",
					i, j, k, ( *DistanceField )[ k * size2 + j * iSize + i ], ( *ClosestPointID )[ k * size2 + j * iSize + i ] );
			} // for i
		} // for j
	} // for k
	
	fclose( FPDF );
	fclose( FPCP );
	fclose( FPDFCP );
	


	/* ���f���_�Q���o�͂��� */
	cout << "�E�E�E���f���̓_�Q���o�́E�E�E" << endl;
	FILE *FPMODELPOINTCLOUD;
	FPMODELPOINTCLOUD = fopen( "Output/DistanceField/PCDModelPointCloud.csv", "w" );
	fprintf( FPMODELPOINTCLOUD, "�ް��`��,2,\nmemo1,\nX,Y,Z,\n" );
	for( i = 0; i < *ModelPointCloudSize; i++ ){
		fprintf( FPMODELPOINTCLOUD, "%f,%f,%f,\n", ( *modelpointcloud )[ i ].Xmodel, ( *modelpointcloud )[ i ].Ymodel, ( *modelpointcloud )[ i ].Zmodel );
	}
	fclose( FPMODELPOINTCLOUD );

#ifdef COLOREVALUATE
 	
	/* �ŋߓ_ID�t��RGB�����o�� */
	cout << "�E�E�E�f�B�X�^���X�t�B�[���h�̐F���o�́E�E�E" << endl;
	FILE *FPCPRGB;
	FPCPRGB = fopen( "Output/CPRGBdata.txt", "w" );
	fprintf( FPCPRGB, "ClosestPointID:R,G,B\n" );
	for( i = 0; i < *ModelPointCloudSize; i++ ){
		fprintf( FPCPRGB, "%d:%d,%d,%d\n", i, ( *DistanceFieldRGB )[ i ].R, ( *DistanceFieldRGB )[ i ].G, ( *DistanceFieldRGB )[ i ].B );
	}
	fclose( FPCPRGB );

#endif // COLOREVALUATE

	delete []CPtemp;
	delete []DFtemp;
	delete []stack;
	delete []x;
}


