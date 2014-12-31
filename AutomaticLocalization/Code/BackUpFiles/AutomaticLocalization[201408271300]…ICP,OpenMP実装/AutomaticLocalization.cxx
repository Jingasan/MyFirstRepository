/***** �f�B�X�^���X�t�B�[���h�̍쐬 *****/


//�y �R�}���h���C������ �z
// 1. �T���Ώۓ_�Q�t�@�C��
// 2. ���f���_�Q�t�@�C��

// �� 1�F
// 1. Input/PointData/CompModel.pcd
// 2. Input/PointData/CompModel.pcd
// �� 2�F
// 1. Input/PointData/LiverID_0002V2U0.01L0.01R20.pcd
// 2. Input/PointData/LiverID_0002V2U0.01L0.01R20.pcd


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
		cerr << argv[ 0 ] << endl << " inputPointCloud1 inputPointCloud2" << endl;
		return EXIT_FAILURE;
	}
	

	/*** �ϐ��̐錾 ***/
	const char * inputFilename1  = argv[ 1 ];			// �T���Ώۓ_�Q�t�@�C����
	const char * inputFilename2  = argv[ 2 ];			// ���f���_�Q�t�@�C����
	
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

	vector< Eigen::Matrix< double, 3, 3 > > ER;
	vector< Eigen::Vector3d > Et;
	vector< double > DFval;
	ER.resize( 0 );
	Et.resize( 0 );
	DFval.resize( 0 );

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
	cout << "--- �y CreateDistanceFieldforPCDModel�֐� �zDF�̍쐬 ---" << endl;
	DWORD CreateDistanceFieldTimeStart = GetTickCount();	// DF �쐬�J�n���Ԃ̎擾
	CreateDistanceFieldforPCDModel(
		DistanceFieldDevideSize,	// DF �̕�����( DF �̃{�N�Z���̕���\ )
		&DistanceField,				// DF
		&ClosestPointID,			// �ŋߓ_�� ID
		&TargetPointCloudSize,		// 3�����_�Q�̐�
		&TargetPointCloud,			// 3�����_�Q
		&TargetInformation,			// 3�����_�Q�̊e����
		cloud1						// ����3�����_�Q
	);	// �f�B�X�^���X�}�b�v�Ɋi�[�����͓̂_����̋����ƑΉ�����_�Ɩ@���x�N�g���̃C���f�b�N�X�ł���	
	DWORD CreateDistanceFieldTimeEnd = GetTickCount();	// DF �쐬�I�����Ԃ̎擾


	/*** �S�T�� ***/
	cout << "--- �y ExhaustiveSearch�֐� �z�S�T�� ---" << endl;
	DWORD ExhaustiveSearchTimeStart = GetTickCount();	// �S�T���J�n���Ԃ̎擾
	ExhaustiveSearch(
		&DistanceField,		// DF
		&TargetInformation,	// �T���Ώۓ_�Q�̊e����
		&ModelInformation,	// ���f���_�Q�̊e����
		&minSumofDistance,	// �T���Ώۓ_�Q�ƃ��f���_�Q�Ƃ̋����]���l�̍ŏ��l
		cloud2,				// ���f���_�Q
		cloud1,				// �T���Ώۓ_�Q
		&ER,				// �S�T�����ʂ̉�]�s��
		&Et,				// �S�T�����ʂ̕��i�x�N�g��
		&DFval				// �S�T�����ʂ� DF �l
		//&R				// ��]�s�� R
	);
	DWORD ExhaustiveSearchTimeEnd = GetTickCount();	// �S�T���I�����Ԃ̎擾






	/* �e�폈�����Ԃ̃R�}���h���C���o�� */
	cout << endl << "�y ResultTime �z" << endl;
	cout << "CreateDistanceFieldTime�F" << (double)( CreateDistanceFieldTimeEnd - CreateDistanceFieldTimeStart ) / 1000 << " sec." << endl;
	cout << "ExhaustiveSearchTime�F" << (double)( ExhaustiveSearchTimeEnd - ExhaustiveSearchTimeStart ) / 1000 << " sec." << endl;

	/* �v���O�����I���̃R�[�� */
	cout << "�y �v���O�����I�� �z" << endl;

	return 0;
}
