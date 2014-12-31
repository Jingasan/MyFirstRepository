/***** Automatic Localization *****/

// �y Program Pipeline �z
// Step1 �f�B�X�^���X�t�B�[���h�̍쐬 
// Step2 �S�T��
// Step3 �œK���FMarquart or ICP


//�y �R�}���h���C������ �z
// 1. �T���Ώۓ_�Q�t�@�C��
// 2. ���f���_�Q�t�@�C��

// �� 1�F
// 1. Input/PointData/CompModel.pcd
// 2. Input/PointData/CompModel.pcd

// �� 2�F�T���Ώۓ_�Q�ƃ��f���_�Q(�ʒu�p���ω���)�̈ʒu���킹
// 1. Input/PointData/LiverID_0002V2U0.01L0.01R20.pcd
// 2. Input/PointData/LiverID_0002V2U0.01L0.01R20.pcd

// �� 3�F�T���Ώۓ_�Q�ƃ��f���_�Q(�ʒu�p���ω��L)�̈ʒu���킹
// 1. Input/PointData/LiverID_0002V2U0.01L0.01R20.pcd
// 2. Input/PointData/R6of72LiverID_0002V2U0.01L0.01R20.pcd

// �� 4�F�T���Ώۓ_�Q�ƃ��f���_�Q(�ʒu�p���ω��L)�̈ʒu���킹
// 1. Output/TransformPointCloud/LiverR20/Ra200Rb205Rc210Tx0Ty0Tz0/0001.pcd
// 2. Input/PointData/ModelData/LiverR20/0001.pcd

// �� 5�FRandomNoise �ǉ���̒T���Ώۓ_�Q�ƃ��f���_�Q�̈ʒu���킹
// 1. Output/NoisyPointData/ModelData/LiverR20/0001N1000.pcd
// 2. Input/PointData/R6of72LiverID_0002V2U0.01L0.01R20.pcd

// �� 6�FGaussianNoise �t����̒T���Ώۓ_�Q�ƃ��f���_�Q�̈ʒu���킹
// 1. Output/PlusGaussianNoisePointData/LiverR20/0001_1Gaussian.pcd
// 2. Input/PointData/R6of72LiverID_0002V2U0.01L0.01R20.pcd

// �� 7�FRandomNoise �ǉ� & GaussianNoise �t����̒T���Ώۓ_�Q�ƃ��f���_�Q�̈ʒu���킹
// 1. Output/NoisyPointData/ModelData/LiverR20/0001_1GaussianN1000.pcd
// 2. Input/PointData/R6of72LiverID_0002V2U0.01L0.01R20.pcd

// �� 8�FRandomNoise �ǉ� & GaussianNoise �t����̒T���Ώۓ_�Q�ƃ��f���_�Q�̈ʒu���킹
// 1. Output/NoisyPointData/ModelData/LiverR20/0001_5GaussianN1000.pcd
// 2. Input/PointData/R6of72LiverID_0002V2U0.01L0.01R20.pcd


/*** �C���N���[�h�t�@�C�� ***/

/* �ݒ�p�w�b�_�t�@�C�� */
#include "Configuration.h"
#include "FunctionDefinition.h"


/*** ���O��Ԃ̐錾 ***/

/* C++ */
using namespace std;


/*** ���C�������� ***/
int main( int argc, char* argv[] ){


	/* �v���O�����J�n�̃R�[�� */
	cout << "�y �v���O�����J�n �z" << endl;

	/* �ő�X���b�h���̎擾 */
	int threadsNum;
#pragma omp parallel
	{ threadsNum = omp_get_num_threads(); }	// { �c } �́u #pragma omp parallel �v�ׂ̗ɋL�q���Ă͂����Ȃ��̂Œ���
	cout << "�ő�X���b�h���F" << threadsNum << endl;

	/* �R�}���h���C�������ǂݍ��ݎ��s�� */
	if( argc < 3 ){
		cerr << "Caution! : �R�}���h���C�������G���[" << endl;
		cerr << "Usage: " << endl;
		cerr << argv[ 0 ] << endl;
		cerr << "1. InputPointCloud1" << endl;
		cerr << "2. InputPointCloud2" << endl;
		system( "pause" );
		return EXIT_FAILURE;
	}else{
		cout << ">> InputTargetData : " << argv[ 1 ] << endl;
		cout << ">> InputModelData : " << argv[ 2 ] << endl;
	}
	

	/*** �ϐ��̐錾 ***/
	const char * inputFilename1 = argv[ 1 ];			// �T���Ώۓ_�Q�t�@�C����
	const char * inputFilename2 = argv[ 2 ];			// ���f���_�Q�t�@�C����
	
	/* DF �쐬�֘A */
	int DistanceFieldDevideSize = DISTANCEFIELDSIZE;	// DF �̕�����( ����l )
	int *DistanceField = NULL;							// DF
	int *ClosestPointID = NULL;  						// �ŋߓ_�� ID ��ۑ�
	int TargetPointCloudSize = 0;						// 3�����_�Q�̐�
	struct ModelPointCloud *TargetPointCloud = NULL;	// 3�����_�Q�i�[�p�ϐ�
	struct ModelInformation TargetInformation;			// 3�����_�Q�̊e����
	int devideSize;										// DF �̕�����( �R�}���h���C������ )

	/* �S�T���֘A */
	struct ModelPointCloud *modelPointCloud = NULL;		// 3�����_�Q�i�[�p�ϐ�
	struct ModelInformation ModelInformation;			// 3�����_�Q�̊e����
	//Eigen::Matrix< double, 3, 3 > *R = NULL;			// 	
	int minSumofDistance = INT_MAX;
	vector< Eigen::Matrix< double, 3, 3 > > ER;	// �S�T�����ʂ̉�]�s��
	vector< Eigen::Vector3d > Et;				// �S�T�����ʂ̕��i�x�N�g��
	vector< double > DFval;						// �S�T�����ʂ�DF���v�l

	/* �������̊m�� */
	ER.resize( 0 ); Et.resize( 0 ); DFval.resize( 0 );

	/* �f�B�X�^���X�t�B�[���h�̕���\�̎w�� �� Default�F100 */
	cout << "�f�B�X�^���X�t�B�[���h�̕���\���w�肵�Ă��������E�E�E" << endl;
	cout << "[ 0 �ȉ� �� ����l:100 �ɐݒ�, 1 �ȏ� �� �w��l�ɐݒ� ]" << endl;
	cout << ">> ";
	cin >> devideSize;
	if( devideSize > 0 ) DistanceFieldDevideSize = devideSize;	// DF �̕���\�̎w��


	/*** �_�Q�̓ǂݍ��݁F PCD �f�[�^ ***/
	
	/* �T���Ώۓ_�Q�f�[�^�̓ǂݍ��� */
	cout << ">>> �T���Ώۓ_�Q( PointXYZ )�̓Ǎ���..." << "\r";
	pcl::PointCloud< pcl::PointXYZ >::Ptr cloud1( new pcl::PointCloud< pcl::PointXYZ > );	// ���͓_�Q( PointXYZ )�̃������m��
	if( pcl::io::loadPCDFile( inputFilename1, *cloud1 ) == -1 ){ cout << ">>> �T���Ώۓ_�Q�f�[�^�̓ǂݍ��ݎ��s" << endl; return 1; }
	cout << ">>> �T���Ώۓ_�Q( PointXYZ )�̓ǂݍ��� OK" << endl;

	/* ���f���_�Q�f�[�^�̓ǂݍ��� */
	cout << ">>> ���f���_�Q( PointXYZ )�̓Ǎ���..." << "\r";
	pcl::PointCloud< pcl::PointXYZ >::Ptr cloud2( new pcl::PointCloud< pcl::PointXYZ > );	// ���͓_�Q( PointXYZ )�̃������m��
	if( pcl::io::loadPCDFile( inputFilename2, *cloud2 ) == -1 ){ cout << ">>> ���f���_�Q�f�[�^�̓ǂݍ��ݎ��s" << endl;return 1; }
	cout << ">>> ���f���_�Q( PointXYZ )�̓ǂݍ��� OK" << endl;


	/*** �f�B�X�^���X�t�B�[���h�̍쐬 ***/
#ifdef CREATEDISTANCEFIELD
	cout << "--- �y CreateDistanceFieldforPCDModel�֐� �zDF�̍쐬 ---" << endl;
	DWORD CreateDistanceFieldTimeStart = GetTickCount();	// DF �쐬�J�n���Ԃ̎擾
	CreateDistanceFieldforPCDModel(
		DistanceFieldDevideSize,	// DF �̕�����( DF �̃{�N�Z���̕���\ )
		&DistanceField,				// DF
		&ClosestPointID,			// �ŋߓ_�� ID
		&TargetPointCloudSize,		// �_�Q��
		&TargetPointCloud,			// �_�Q
		&TargetInformation,			// �_�Q�̊e����
		cloud1						// ���͓_�Q
	);	// �f�B�X�^���X�}�b�v�Ɋi�[�����͓̂_����̋����ƑΉ�����_�Ɩ@���x�N�g���̃C���f�b�N�X�ł���	
	DWORD CreateDistanceFieldTimeEnd = GetTickCount();	// DF �쐬�I�����Ԃ̎擾
#endif

	/*** �S�T�� ***/
#ifdef EXHAUSTIVESEARCH
	cout << "--- �y ExhaustiveSearch�֐� �z�S�T�� ---" << endl;
	DWORD ExhaustiveSearchTimeStart = GetTickCount();	// �S�T���J�n���Ԃ̎擾
	ExhaustiveSearch(
		DistanceFieldDevideSize,	// DF �̕�����( DF �̃{�N�Z���̕���\ )
		&DistanceField,				// DF
		&TargetInformation,			// �T���Ώۓ_�Q�̊e����
		&ModelInformation,			// ���f���_�Q�̊e����
		&minSumofDistance,			// �T���Ώۓ_�Q�ƃ��f���_�Q�Ƃ̋����]���l�̍ŏ��l
		cloud2,						// ���f���_�Q
		cloud1,						// �T���Ώۓ_�Q
		&ER,						// �S�T�����ʂ̉�]�s��
		&Et,						// �S�T�����ʂ̕��i�x�N�g��
		&DFval						// �S�T�����ʂ� DF �l
		//&R	// ��]�s�� R
	);
	DWORD ExhaustiveSearchTimeEnd = GetTickCount();	// �S�T���I�����Ԃ̎擾
#endif

	/*** �}�[�J�[�g�@�ɂ��œK�� ***/
#ifdef MARQUARDT
	cout << "--- �y LevMarOptimization�֐� �z�œK�� ---" << endl;
	DWORD LevMarTimeStart = GetTickCount();	// LevenbergMarquardt�@�ɂ��œK���̊J�n���Ԃ̎擾
	LevMarOptimization(
		&DistanceField,				// DF
		&ClosestPointID,			// �ŋߓ_�� ID
		&TargetInformation,			// �T���Ώۓ_�Q�̊e����
		&ModelInformation,			// ���f���_�Q�̊e����
		cloud2,						// ���f���_�Q
		cloud1,						// �T���Ώۓ_�Q
		&ER,						// �S�T�����ʂ̉�]�s��
		&Et,						// �S�T�����ʂ̕��i�x�N�g��
		&DFval						// �S�T�����ʂ� DF �l
	);
	DWORD LevMarTimeEnd = GetTickCount();	// LevenbergMarquardt�@�ɂ��œK���̏I�����Ԃ̎擾
#endif

	/*** ICP ��p����2�_�Q�̍œK�� ***/
#ifdef ITERATIVECLOSESTPOINT
	cout << "--- �y ICPOptimization�֐� �z�œK�� ---" << endl;
	pcl::PointCloud< pcl::PointXYZRGB >::Ptr newCloudM( new pcl::PointCloud< pcl::PointXYZRGB > );		// �ʒu���킹��̃��f���_�Q
	pcl::PointCloud< pcl::PointXYZRGB >::Ptr mergedIcpCloud( new pcl::PointCloud< pcl::PointXYZRGB > );	// ICP ��̓����_�Q
	DWORD ICPTimeStart = GetTickCount();	// ICP�A���S���Y���ɂ��œK���̊J�n���Ԃ̎擾
	ICPOptimization(
		cloud2,						// ���f���_�Q
		cloud1,						// �T���Ώۓ_�Q
		newCloudM,					// �œK����̃��f���_�Q
		mergedIcpCloud,				// �œK����̃��f���_�Q�ƒT���Ώۓ_�Q�̓�������
		&TargetInformation,			// �T���Ώۓ_�Q�̊e����
		&ModelInformation,			// ���f���_�Q�̊e����
		&ER,						// �S�T�����ʂ̉�]�s��
		&Et							// �S�T�����ʂ̕��i�x�N�g��
	);
	DWORD ICPTimeEnd = GetTickCount();	// ICP�A���S���Y���ɂ��œK���̏I�����Ԃ̎擾
#endif



	/* �e�폈�����Ԃ̃R�}���h���C���o�� */
	cout << endl << "�y ResultTime �z" << endl;
	double CreateDistanceFieldTime = -1; double ExhaustiveSearchTime = -1; double LevMarTime = -1; double ICPTime = -1;
#ifdef CREATEDISTANCEFIELD
	CreateDistanceFieldTime = (double)( CreateDistanceFieldTimeEnd - CreateDistanceFieldTimeStart ) / 1000;
	cout << "CreateDistanceFieldTime�F" << CreateDistanceFieldTime << " sec." << endl;
#endif
#ifdef EXHAUSTIVESEARCH
	ExhaustiveSearchTime = (double)( ExhaustiveSearchTimeEnd - ExhaustiveSearchTimeStart ) / 1000;
	cout << "ExhaustiveSearchTime�F" << ExhaustiveSearchTime << " sec." << endl;
#endif
#ifdef MARQUARDT
	LevMarTime = (double)( LevMarTimeEnd - LevMarTimeStart ) / 1000;
	cout << "LevenbergMarquardtTime�F" << LevMarTime << " sec." << endl;
#endif
#ifdef ITERATIVECLOSESTPOINT
	ICPTime = (double)( ICPTimeEnd - ICPTimeStart ) / 1000;
	cout << "ICPTime�F" << ICPTime << " sec." << endl;
#endif
#ifdef OUTPUTEXECUTIONTIME
	cout << ">> ���s���ʏo�͒�" << "\r";
	FILE *outputFp1;
	if( ( outputFp1 = fopen( ExecutionTimeFileName, "w" ) ) == NULL ){ cout << "Caution!: " << ExecutionTimeFileName << " open error" << endl; return 1; }
	fprintf( outputFp1, "CreateDistanceFieldTime[sec],%f\nExhaustiveSearchTime[sec],%f\nLevenbergMarquardtTime[sec],%f\nICPTime[sec],%f\n", CreateDistanceFieldTime, ExhaustiveSearchTime, LevMarTime, ICPTime );
	fclose( outputFp1 );
	cout << ">> ���s���ʏo�� OK!" << endl;
#endif

	/* �v���O�����I���̃R�[�� */
	cout << endl << "�y �v���O�����I�� �z" << endl;

	return 0;
}
