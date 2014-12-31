/***** �S�T�� *****/


/*** �C���N���[�h�t�@�C�� ***/

/* �ݒ�p�w�b�_�t�@�C�� */
#include "Configuration.h"
#include "FunctionDefinition.h"


/*** ���O��Ԃ̐錾 ***/

/* C++ */
using namespace std;


/*** �S�T�� ***/
int ExhaustiveSearch(
	int** DistanceField,							// DF
	struct ModelInformation* targetInformation,		// �T���Ώۓ_�Q�̊e����
	struct ModelInformation* modelInformation,		// ���f���_�Q�̊e����
	int* minSumofDistance,							// �T���Ώۓ_�Q�ƃ��f���_�Q�Ƃ̋����]�����v�l�̍ŏ��l
	pcl::PointCloud< pcl::PointXYZ >::Ptr cloudM,	// ���f���_�Q
	pcl::PointCloud< pcl::PointXYZ >::Ptr cloudT,	// �T���Ώۓ_�Q
	vector< Eigen::Matrix< double, 3, 3 > > *ER,		// �S�T�����ʂ̉�]�s��
	vector< Eigen::Vector3d > *Et,					// �S�T�����ʂ̕��i�x�N�g��
	vector< double > *DFval							// �S�T�����ʂ� DF �l
	//Eigen::Matrix< double, 3, 3 > **R				// ��]�s�� R
){
	
	/*** �ϐ��̐錾 ***/

	/* �S�T���֘A */
	FILE *inputFp1;	// �T���v�����O���ꂽ�l�����̓��̓t�@�C��
	FILE *inputFp2;	// �T���v�����O���ꂽ�l�����ƑΉ������]�p�ƃ����̓��̓t�@�C��
	vector< vector< float > > quaternions;	// �T���v�����O���ꂽ�l����
	vector< float > quaternion;				// �l����
	vector< vector< float > > degrees;		// �T���v�����O���ꂽ��]�p�ƃ���
	vector< float > degree;					// ��]�p�ƃ���
	int sampX = SAMP_X;	// X �����̕��i�̃T���v�����O��
	int sampY = SAMP_Y;	// Y �����̕��i�̃T���v�����O��
	int sampZ = SAMP_Z;	// Z �����̕��i�̃T���v�����O��
	int sampRLevel = SAMP_R_LEVEL;	// ��]��Ԃ̋ϓ��ȃT���v�����O�̉𑜓x���x�� �� 0, 1, 2, 3 ����w��
	int lowerX = - targetInformation->deltaX / ( 2 * TOMETER );	// X �����̕��i�̒T���͈͂̉���
	int lowerY = - targetInformation->deltaY / ( 2 * TOMETER );	// Y �����̕��i�̒T���͈͂̉���
	int lowerZ = - targetInformation->deltaZ / ( 2 * TOMETER );	// Z �����̕��i�̒T���͈͂̉���
	int upperX = targetInformation->deltaX / ( 2 * TOMETER );	// X �����̕��i�̒T���͈͂̏��
	int upperY = targetInformation->deltaY / ( 2 * TOMETER );	// Y �����̕��i�̒T���͈͂̏��
	int upperZ = targetInformation->deltaZ / ( 2 * TOMETER );	// Z �����̕��i�̒T���͈͂̏��
	lowerX = lowerX / LIMIT_X;	// X �����̕��i�̒T���͈͂̉����l�̍i�荞��
	lowerY = lowerY / LIMIT_Y;	// Y �����̕��i�̒T���͈͂̉����l�̍i�荞�� 
	lowerZ = lowerZ / LIMIT_Z;	// Z �����̕��i�̒T���͈͂̉����l�̍i�荞��
	upperX = upperX / LIMIT_X;	// X �����̕��i�̒T���͈͂̏���l�̍i�荞��
	upperY = upperY / LIMIT_Y;	// Y �����̕��i�̒T���͈͂̏���l�̍i�荞��
	upperZ = upperZ / LIMIT_Z;	// Z �����̕��i�̒T���͈͂̏���l�̍i�荞��
	int SumofDistance;			// DF ��p�����T���Ώۓ_�Q�ƃ��f���_�Q�Ƃ̋����]���l
	double distanceMeans;		// 1�_������̋����]���l

	/* �T���Ώۓ_�Q�ƃ��f���_�Q�Ƃ̋����]���֘A */
	int iSize = targetInformation->iSize;			// DF �� i �����̃T�C�Y
	int jSize = targetInformation->jSize;			// DF �� j �����̃T�C�Y
	int kSize = targetInformation->kSize;			// DF �� k �����̃T�C�Y
	int ijSize = iSize * jSize;						// DF �� i, j �����̖ʐ�
	double targetXSize = targetInformation->deltaX;	// �T���Ώۓ_�Q�� X �����̍ő啝 
	double targetYSize = targetInformation->deltaY;	// �T���Ώۓ_�Q�� Y �����̍ő啝
	double targetZSize = targetInformation->deltaZ;	// �T���Ώۓ_�Q�� Z �����̍ő啝
	double gravX = targetInformation->gravX;	// �T���Ώۓ_�Q�̏d�S X
	double gravY = targetInformation->gravY;	// �T���Ώۓ_�Q�̏d�S Y
	double gravZ = targetInformation->gravZ;	// �T���Ώۓ_�Q�̏d�S Z
	double mGravX;	// ���f���_�Q�̏d�S X
	double mGravY;	// ���f���_�Q�̏d�S Y
	double mGravZ;	// ���f���_�Q�̏d�S Z
	int sum = 0;	// �T���Ώۓ_�Q�ƃ��f���_�Q�Ƃ̋����]���l�̍��v

	int minRID;
	Eigen::Matrix< double, 3, 3 > minR;
	Eigen::Vector3d minDegree;
	Eigen::Vector3d mint;
	Eigen::Matrix< double, 3, 3 > *R;	// �ϓ��ɃT���v�����O���ꂽ��]�s��
	Eigen::Vector3d t;					// ���i�x�N�g��
	Eigen::Vector3d p;					// ��]�O�_�Q�̈ꎞ�i�[�p�z��
	Eigen::Vector3d rp;					// ��]��_�Q�̈ꎞ�i�[�p�z��

	/* ������ */
	modelInformation->Xmin = INFINITY;
	modelInformation->Ymin = INFINITY;
	modelInformation->Zmin = INFINITY;
	modelInformation->Xmax = -INFINITY;
	modelInformation->Ymax = -INFINITY;
	modelInformation->Zmax = -INFINITY;

	/* �������̊m�� */
	quaternions.resize( 0 );
	quaternion.resize( 4 );
	degrees.resize( 0 );
	degree.resize( 3 );


	/*** ���f���_�Q�̏d�S�̎Z�o ***/

	/* �_�Q�̒[�_��ۑ� */
	for( size_t i = 0; i < cloudM->points.size(); i++ ){
		
		/* X, Y, Z �̍ŏ��l��ݒ� */
		if( cloudM->points[ i ].x * TOMETER < modelInformation->Xmin ) modelInformation->Xmin = cloudM->points[ i ].x * TOMETER;
		if( cloudM->points[ i ].y * TOMETER < modelInformation->Ymin ) modelInformation->Ymin = cloudM->points[ i ].y * TOMETER;
		if( cloudM->points[ i ].z * TOMETER < modelInformation->Zmin ) modelInformation->Zmin = cloudM->points[ i ].z * TOMETER;
		
		/* X, Y, Z �̍ő�l��ݒ� */
		if( cloudM->points[ i ].x * TOMETER > modelInformation->Xmax ) modelInformation->Xmax = cloudM->points[ i ].x * TOMETER;
		if( cloudM->points[ i ].y * TOMETER > modelInformation->Ymax ) modelInformation->Ymax = cloudM->points[ i ].y * TOMETER;
		if( cloudM->points[ i ].z * TOMETER > modelInformation->Zmax ) modelInformation->Zmax = cloudM->points[ i ].z * TOMETER;
	}

	/* �_�Q�̏d�S�̎Z�o */ 
	mGravX = ( modelInformation->Xmax + modelInformation->Xmin ) / 2;
	mGravY = ( modelInformation->Ymax + modelInformation->Ymin ) / 2;
	mGravZ = ( modelInformation->Zmax + modelInformation->Zmin ) / 2;

	/* �_�Q�̏d�S�����f�����Ƃ��Ċi�[ */
	modelInformation->gravX = mGravX;
	modelInformation->gravY = mGravY;
	modelInformation->gravZ = mGravZ;

	/* �_�Q�̏d�S�����_�ɐݒ� */
	modelInformation->Xmin -= mGravX; modelInformation->Xmax -= mGravX;
	modelInformation->Ymin -= mGravY; modelInformation->Ymax -= mGravY;
	modelInformation->Zmin -= mGravZ; modelInformation->Zmax -= mGravZ;
	
	/* �_�Q�̍ő啝�̎Z�o */
	modelInformation->deltaX = modelInformation->Xmax - modelInformation->Xmin;
	modelInformation->deltaY = modelInformation->Ymax - modelInformation->Ymin;
	modelInformation->deltaZ = modelInformation->Zmax - modelInformation->Zmin;
    
	/* �ł��傫���l�����_�Q���̎Z�o X���� or Y���� or Z���� */
	if( modelInformation->deltaX > modelInformation->deltaY ){
		if( modelInformation->deltaX > modelInformation->deltaZ ){
			modelInformation->max = modelInformation->deltaX;
		}else{
			modelInformation->max = modelInformation->deltaZ;
		}
	}else{
		if( modelInformation->deltaY > modelInformation->deltaZ ){
			modelInformation->max = modelInformation->deltaY;
		}else{
			modelInformation->max = modelInformation->deltaZ;
		}
	}


	/*** �T���v�����O���ꂽ�l�����Ƃ��̉�]�p�̓ǂݍ��݂Ɖ�]�s��ւ̕ϊ� ***/

	/* �t�@�C���̓W�J */
	if( sampRLevel == 0 ){
		if( ( inputFp1 = fopen( QuaternionFileName1, "r" ) ) == NULL ){ cout << "Caution!: " << QuaternionFileName1 << " open error" << endl; return 1; }
		if( ( inputFp2 = fopen( DegreeFileName1, "r" ) ) == NULL ){ cout << "Caution!: " << DegreeFileName1 << " open error" << endl; return 1; }
	}else if( sampRLevel == 1 ){
		if( ( inputFp1 = fopen( QuaternionFileName2, "r" ) ) == NULL ){ cout << "Caution!: " << QuaternionFileName2 << " open error" << endl; return 1; }
		if( ( inputFp2 = fopen( DegreeFileName2, "r" ) ) == NULL ){ cout << "Caution!: " << DegreeFileName2 << " open error" << endl; return 1; }
	}else if( sampRLevel == 2 ){
		if( ( inputFp1 = fopen( QuaternionFileName3, "r" ) ) == NULL ){ cout << "Caution!: " << QuaternionFileName3 << " open error" << endl; return 1; }
		if( ( inputFp2 = fopen( DegreeFileName3, "r" ) ) == NULL ){ cout << "Caution!: " << DegreeFileName3 << " open error" << endl; return 1; }
	}else if( sampRLevel == 3 ){
		if( ( inputFp1 = fopen( QuaternionFileName4, "r" ) ) == NULL ){ cout << "Caution!: " << QuaternionFileName4 << " open error" << endl; return 1; }
		if( ( inputFp2 = fopen( DegreeFileName4, "r" ) ) == NULL ){ cout << "Caution!: " << DegreeFileName4 << " open error" << endl; return 1; }
	}else{ cout << "Caution!: Setting SampRLevel error" << endl; return 1; }

	/* �t�@�C�����e�̓ǂݍ��� */
	while( fscanf( inputFp1, "%f,%f,%f,%f", &quaternion[ 0 ], &quaternion[ 1 ], &quaternion[ 2 ], &quaternion[ 3 ] ) != EOF ) quaternions.push_back( quaternion );	// 2�����z��̖����ւ�1�����z��̒ǉ�
	while( fscanf( inputFp2, "%f,%f,%f", &degree[ 0 ], &degree[ 1 ], &degree[ 2 ] ) != EOF ) degrees.push_back( degree );	// 2�����z��̖����ւ�1�����z��̒ǉ�
	
	/* �������̉�� */
	fclose( inputFp1 );
	fclose( inputFp2 );

#ifdef COMMENT
	for( int i = 0; i < quaternions.size(); i++ ){
		cout << "Quaternion " << i << " : ";
		for( int j = 0; j < quaternions[ i ].size(); j++ ){ cout << quaternions[ i ][ j ] << " "; }
		cout << endl << "Degree: ";
		for( int j = 0; j < degrees[ i ].size(); j++ ){ cout << degrees[ i ][ j ] << " "; }
		cout << endl << endl;
	}
#endif

	/* ��]�s��̃������m�� */
	R = new Eigen::Matrix< double, 3, 3 >[ quaternions.size() ];

	/* ��]�s��̕��� */
	for( int i = 0; i < quaternions.size(); i++ ){

		double element0 = (double)quaternions[ i ][ 0 ];	// �l������1�ڂ̗v�f
		double element1 = (double)quaternions[ i ][ 1 ];	// �l������2�ڂ̗v�f 
		double element2 = (double)quaternions[ i ][ 2 ];	// �l������3�ڂ̗v�f
		double element3 = (double)quaternions[ i ][ 3 ];	// �l������4�ڂ̗v�f
		double element0S = pow( element0, 2 );	// �l������1�ڂ̗v�f��2��
		double element1S = pow( element1, 2 );	// �l������2�ڂ̗v�f��2��
		double element2S = pow( element2, 2 );	// �l������3�ڂ̗v�f��2��
		double element3S = pow( element3, 2 );	// �l������4�ڂ̗v�f��2��

		/* �l�����`������̉�]�s��̕��� */
		R[ i ]( 0, 0 ) = element0S + element1S - element2S - element3S;		// 1�s1��ڂ̗v�f
		R[ i ]( 0, 1 ) = 2 * ( element1 * element2 - element0 * element3 );	// 1�s2��ڂ̗v�f
		R[ i ]( 0, 2 ) = 2 * ( element1 * element3 + element0 * element2 );	// 1�s3��ڂ̗v�f
		R[ i ]( 1, 0 ) = 2 * ( element1 * element2 + element0 * element3 );	// 2�s1��ڂ̗v�f
		R[ i ]( 1, 1 ) = element0S - element1S + element2S - element3S;		// 2�s2��ڂ̗v�f
		R[ i ]( 1, 2 ) = 2 * ( element2 * element3 - element0 * element1 );	// 2�s3��ڂ̗v�f
		R[ i ]( 2, 0 ) = 2 * ( element1 * element3 - element0 * element2 );	// 3�s1��ڂ̗v�f
		R[ i ]( 2, 1 ) = 2 * ( element2 * element3 + element0 * element1 );	// 3�s2��ڂ̗v�f
		R[ i ]( 2, 2 ) = element0S - element1S - element2S + element3S;		// 3�s3��ڂ̗v�f

#ifdef COMMENT
		cout << endl << "Index: " << i << endl << R[ i ] << endl << R[ i ].determinant() << endl;
#endif
	}


	/*** �T���Ώۓ_�Q�ƃ��f���_�Q�Ƃ̋����]�� ***/
	cout << "--- �T���Ώۓ_�Q�ƃ��f���_�Q�Ƃ̋����]�� ---" << endl;

	/* ���f���_�Q�̉�] */
	for( int r = 0; r < quaternions.size(); r++ ){

		/* ��]��̃��f���_�Q */
		pcl::PointCloud< pcl::PointXYZ >::Ptr rotatedPointCloud( new pcl::PointCloud< pcl::PointXYZ > );
		pcl::PointXYZ temp1;	// �_���W�̈ꎞ�i�[�p�ϐ�

		cout << "RotationID: " << r << endl << "Degree: ";
		for( int i = 0; i < degrees[ r ].size(); i++ ){ cout << degrees[ r ][ i ] << " "; }
		cout << endl << "R: " << endl << R[ r ] << endl;

		/* DF �̍��W�n�ւ̕ϊ�, ��]��̃��f���_�Q */
		for( int i = 0; i < cloudM->size(); i++ ){
			
			/* ���f���_�Q�� DF ���W�n�֕ϊ��FDF ���W�n�͓_�Q�̏d�S�����_ */
			p( 0 ) = cloudM->points[ i ].x - ( mGravX / (double)TOMETER );
			p( 1 ) = cloudM->points[ i ].y - ( mGravY / (double)TOMETER );
			p( 2 ) = cloudM->points[ i ].z - ( mGravZ / (double)TOMETER );
			
			/* DF �̍��W�n���_�𒆐S�Ƃ������f���_�Q�̉�] */
			rp = R[ r ] * p;
			
			/* ��]��̓_�Q�̊i�[ */
			temp1.x = rp( 0 );
			temp1.y = rp( 1 );
			temp1.z = rp( 2 );
			//temp1.x = p( 0 );
			//temp1.y = p( 1 );
			//temp1.z = p( 2 );
			rotatedPointCloud->points.push_back( temp1 );
		}
		rotatedPointCloud->width = rotatedPointCloud->points.size();
		rotatedPointCloud->height = 1;

		
		for( int tz = lowerZ; tz < upperZ; tz += sampZ ){
			for( int ty = lowerY; ty < upperY; ty += sampY ){
				for( int tx = lowerX; tx < upperX; tx += sampX ){
					//int tx = 0; int ty = 0; int tz = 0;

					/* DF �̍��W�n�ւ̕ϊ�, ��]���s�ړ���̃��f���_�Q */
					pcl::PointCloud< pcl::PointXYZ >::Ptr transPointCloud( new pcl::PointCloud< pcl::PointXYZ > );

					/* ���f���_�Q�� DF �̍��W�n�ւ̕ϊ��Ɖ�]���s�ړ� */
					t( 0 ) = (double)tx; t( 1 ) = (double)ty; t( 2 ) = (double)tz;
					for( int i = 0; i < rotatedPointCloud->size(); i++ ){
					
						temp1.x = rotatedPointCloud->points[ i ].x + t( 0 );
						temp1.y = rotatedPointCloud->points[ i ].y + t( 1 );
						temp1.z = rotatedPointCloud->points[ i ].z + t( 2 );
						//temp1.x = ( cloudM->points[ i ].x - ( mGravX / (double)TOMETER ) ) + t( 0 );
						//temp1.y = ( cloudM->points[ i ].y - ( mGravY / (double)TOMETER ) ) + t( 1 );
						//temp1.z = ( cloudM->points[ i ].z - ( mGravZ / (double)TOMETER ) ) + t( 2 );
						transPointCloud->points.push_back( temp1 );
					}
					transPointCloud->width = transPointCloud->points.size();
					transPointCloud->height = 1;

				
					/*** �T���Ώۓ_�Q�ƃ��f���_�Q�Ƃ̋����]���l�̎Z�o ***/
					sum = 0;
					for( int n = 0; n < transPointCloud->size(); n++ ){

						/* ���f���_�Q�� DF ���ɂ���T���Ώۓ_�Q�̍��W�n�ֈړ� */
						transPointCloud->points[ n ].x = transPointCloud->points[ n ].x * (double)TOMETER;
						transPointCloud->points[ n ].y = transPointCloud->points[ n ].y * (double)TOMETER;
						transPointCloud->points[ n ].z = transPointCloud->points[ n ].z * (double)TOMETER;

						/* ���f���_�Q�� DF ��ł̍��W�l�̎Z�o */
						int i = (int)( ( (double)iSize * ( transPointCloud->points[ n ].x + targetXSize ) / ( 2.0 * targetXSize ) ) + 0.5 );
						int j = (int)( ( (double)jSize * ( transPointCloud->points[ n ].y + targetYSize ) / ( 2.0 * targetYSize ) ) + 0.5 );
						int k = (int)( ( (double)kSize * ( transPointCloud->points[ n ].z + targetZSize ) / ( 2.0 * targetZSize ) ) + 0.5 );

						/* ���f���_�Q�ɂ����錻�ݎQ�Ɠ_�� DF ���ɂ���ꍇ */
						if( ( ( i >= 0 ) && ( i < iSize - 1 ) ) && ( ( j >= 0 ) && ( j < jSize - 1 ) ) && ( ( k >= 0 ) && ( k < kSize - 1 ) ) ){
						//cout << "Point( " << transPointCloud->points[ n ].x << ", " << transPointCloud->points[ n ].y << ", " << transPointCloud->points[ n ].z << " ): " << endl;
						//cout << "DF( " << i << ", " << j << ", " << k << " ): " << ( *DistanceField )[ k * ijSize + j * iSize + i ] << endl;

							/* �T���Ώۓ_�Q�ƃ��f���_�Q�Ƃ̋����]���l�̍��v�l���Z�o */
							sum += ( *DistanceField )[ k * ijSize + j * iSize + i ];
						}
						
						/* ���f���_�Q�ɂ����錻�ݎQ�Ɠ_�� DF �O�ɂ���ꍇ */
						else{
							sum += 999;	// DF �l�̎擾���s�\�Ȃ̂ŁA���̋����l�Ƃ��� 999 ���J�E���g
						}

						/* �ŏ����v�����]���l�������݂̍��v�����]���l�̕����傫���ꍇ */
						if( sum > *minSumofDistance || sum > ( INT_MAX / 10 ) ) break;
					}
#ifdef COMMENTa
					cout << "DFSize: " << iSize << " " << jSize << " " << kSize << endl;
					cout << "TargetSize: " << targetXSize << " " << targetYSize << " " << targetZSize << endl;
					cout << "TargetGravity: " << gravX << " " << gravY << " " << gravZ << endl;
					cout << "ModelGravity: " << mGravX << " " << mGravY << " " << mGravZ << endl;
					cout << "ModelSize: " << modelInformation->deltaX << " " << modelInformation->deltaY << " " << modelInformation->deltaZ << endl;
					cout << "SumofDistance: " << sum << endl;
#endif

					/* �T���Ώۓ_�Q�ƃ��f���_�Q�Ƃ̋����]���l�̍��v */
					SumofDistance = sum;

					/* �T���Ώۓ_�Q�ƃ��f���_�Q�Ƃ̋����]���l�̍ŏ��l�̍X�V */
					if( SumofDistance < *minSumofDistance ){
					
						*minSumofDistance = SumofDistance; 

						/* �ŏ������]���l���Ƃ���i�x�N�g�� */
						mint( 0 ) = tx; mint( 1 ) = ty; mint( 2 ) = tz;
						minR = R[ r ];
						minDegree( 0 ) = degrees[ r ][ 0 ];
						minDegree( 1 ) = degrees[ r ][ 1 ];
						minDegree( 2 ) = degrees[ r ][ 2 ];
						minRID = r;

						/* 1�_������̋����]���l */
						distanceMeans = (double)( *minSumofDistance ) / (double)cloudM->size();	// 1�_������ǂ̂��炢����Ă��邩
#ifdef COMMENT
						if( *minSumofDistance < 10000 ){
							cout << "�ŏ����v�����]���l: " << *minSumofDistance << endl;
							cout << "t: " << mint( 0 ) << ", " << mint( 1 ) << ", " << mint( 2 ) << endl;
							cout << "R ID: " << minRID << endl;
							cout << "R: " << minR( 0, 0 ) << "\t" << minR( 0, 1 ) << "\t" << minR( 0, 2 ) << endl;
							cout << "   " << minR( 1, 0 ) << "\t" << minR( 1, 1 ) << "\t" << minR( 1, 2 ) << endl;
							cout << "   " << minR( 2, 0 ) << "\t" << minR( 2, 1 ) << "\t" << minR( 2, 2 ) << endl;
							cout << "Degree: " << minDegree( 0 ) << ", " << minDegree( 1 ) << ", " << minDegree( 2 ) << endl;
							cout << "DistanceMeans: " << distanceMeans << endl;
						}
#endif
					}
				}
			}
		}

#ifdef COMMENT
		cout << "�ŏ����v�����]���l: " << *minSumofDistance << endl;
		cout << "t: " << mint( 0 ) << ", " << mint( 1 ) << ", " << mint( 2 ) << endl;
		cout << "DistanceMeans: " << distanceMeans << endl << endl;
#endif
	}


	/* �S�T�����ʂ̃R�}���h���C���o�� */
	cout << "----- �S�T������ -----" << endl;
	cout << "�ŏ����v�����]���l: " << *minSumofDistance << endl;
	cout << "t: " << mint( 0 ) << ", " << mint( 1 ) << ", " << mint( 2 ) << endl;
	cout << "R ID: " << minRID << endl;
	cout << "R: " << minR( 0, 0 ) << "\t" << minR( 0, 1 ) << "\t" << minR( 0, 2 ) << endl;
	cout << "   " << minR( 1, 0 ) << "\t" << minR( 1, 1 ) << "\t" << minR( 1, 2 ) << endl;
	cout << "   " << minR( 2, 0 ) << "\t" << minR( 2, 1 ) << "\t" << minR( 2, 2 ) << endl;
	cout << "Degree: " << minDegree( 0 ) << ", " << minDegree( 1 ) << ", " << minDegree( 2 ) << endl;
	cout << "DistanceMeans: " << distanceMeans << endl << endl;


#ifdef ESRESULT

	/*** �S�T�����ʂ̏o�� ***/
	pcl::PointCloud< pcl::PointXYZRGB >::Ptr targetPointCloud( new pcl::PointCloud< pcl::PointXYZRGB > );
	pcl::PointCloud< pcl::PointXYZRGB >::Ptr modelPointCloud( new pcl::PointCloud< pcl::PointXYZRGB > );
	pcl::PointCloud< pcl::PointXYZRGB >::Ptr transformedPointCloud( new pcl::PointCloud< pcl::PointXYZRGB > );
	pcl::PointXYZRGB temp2;	// �_���W�̈ꎞ�i�[�p�ϐ�

	/* �T���Ώۓ_�Q */
	for( int i = 0; i < cloudT->size(); i++ ){
			
		/* �T���Ώۓ_�Q�� DF ���W�n�֕ϊ��FDF ���W�n�͓_�Q�̏d�S�����_ */
		temp2.x = cloudT->points[ i ].x - ( gravX / (double)TOMETER );
		temp2.y = cloudT->points[ i ].y - ( gravY / (double)TOMETER );
		temp2.z = cloudT->points[ i ].z - ( gravZ / (double)TOMETER );
		temp2.r = 0;
		temp2.g = 255;
		temp2.b = 0;
		targetPointCloud->points.push_back( temp2 );
	}
	targetPointCloud->width = targetPointCloud->points.size();
	targetPointCloud->height = 1;

	/* ���f���_�Q */
	for( int i = 0; i < cloudM->size(); i++ ){
			
		/* ���f���_�Q�� DF ���W�n�֕ϊ��FDF ���W�n�͓_�Q�̏d�S�����_ */
		temp2.x = cloudM->points[ i ].x - ( mGravX / (double)TOMETER );
		temp2.y = cloudM->points[ i ].y - ( mGravY / (double)TOMETER );
		temp2.z = cloudM->points[ i ].z - ( mGravZ / (double)TOMETER );
		temp2.r = 255;
		temp2.g = 0;
		temp2.b = 0;
		modelPointCloud->points.push_back( temp2 );
	}
	modelPointCloud->width = cloudM->points.size();
	modelPointCloud->height = 1;

	/* ���f���_�Q�̉�]���s�ړ� */
	for( int i = 0; i < cloudM->size(); i++ ){
		
		p( 0 ) = modelPointCloud->points[ i ].x;
		p( 1 ) = modelPointCloud->points[ i ].y;
		p( 2 ) = modelPointCloud->points[ i ].z;
		
		rp = R[ minRID ] * p + mint;

		temp2.x = rp( 0 );
		temp2.y = rp( 1 );
		temp2.z = rp( 2 );
		temp2.r = modelPointCloud->points[ i ].r;
		temp2.g = modelPointCloud->points[ i ].g;
		temp2.b = modelPointCloud->points[ i ].b;
		transformedPointCloud->points.push_back( temp2 );
	}
	transformedPointCloud->width = modelPointCloud->points.size();
	transformedPointCloud->height = 1;

	/* �_�Q�̓��� + ���� + �_�Q�o�͕ۑ� */
	mergedPointDataViewerRGB( targetPointCloud, modelPointCloud, BeforeExhaustiveSearchPCDFileName, BeforeExhaustiveSearchPLYFileName );
	mergedPointDataViewerRGB( targetPointCloud, transformedPointCloud, AfterExhaustiveSearchPCDFileName, AfterExhaustiveSearchPLYFileName );

#endif

#ifdef ITERATIVECLOSESTPOINT
	
	/*** ICP ��p����2�_�Q�̍œK�� ***/
	pcl::PointCloud< pcl::PointXYZRGB >::Ptr newCloudM( new pcl::PointCloud< pcl::PointXYZRGB > );		// �ʒu���킹��̃��f���_�Q
	pcl::PointCloud< pcl::PointXYZRGB >::Ptr mergedIcpCloud( new pcl::PointCloud< pcl::PointXYZRGB > );	// ICP ��̓����_�Q
	
	/* ICP */
	ICPOptomizationRGB( transformedPointCloud, targetPointCloud, newCloudM, mergedIcpCloud );

#endif


	return 0;
}