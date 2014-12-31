/***** 3�����_�Q���̍팸 *****/

//�y �R�}���h���C������ �z
// 1. ���͓_�Q�t�@�C����
// �� PCD �f�[�^
// 2. �o�͓_�Q�t�@�C����
// �� PCD �f�[�^
// 3. �_�Q�t�@�C���^�C�v
// �� 1:PCD( PointXYZRGB ), 2:PCD( PointXYZ )
// 4. �{�N�Z���O���b�h�T�C�Y
// �� float �^ [�P��:m]

// ��F
// Output/EdgePointData/ID_0002_LiverV31U10L1.pcd Output/EdgePointData/ID_0002_LiverV31U10L1R10.pcd 1 10.0


/*** �C���N���[�h�t�@�C�� ***/

/* PCL 1.6.0 */
#include <pcl/point_types.h>		// �_�Q�̃f�[�^�^

/* �쐬�w�b�_�t�@�C�� */
#include "FunctionDefinition.h"	// �֐��̒�`


/*** ���O��Ԃ̐錾 ***/
using namespace std;


/*** ���C�������� ***/
int main( int argc, char* argv[] ){


	/*** �v���O�����J�n�̃R�[�� ***/
	cout << "----- VoxelGrid �ɂ��_�Q���̍팸�J�n -----" << endl;


	/* �R�}���h���C�������ǂݍ��ݎ��s�� */
	if( argc < 5 ){
		cerr << "Caution! : �R�}���h���C�������G���[" << endl;
		cerr << "Usage: " << endl;
		cerr << argv[ 0 ] << endl << "1. inputFileName"
			<< endl << "2. outputFileName"
			<< endl << "3. KindOfPointCloud �� 1:PCD 2:PLY"
			<< endl << "[ 4. VoxelGridSize ]" << endl;
		return EXIT_FAILURE;
	}


	/* �ϐ��̐錾 */
	const char * inputFilename = argv[ 1 ];		// ���̓t�@�C����
	const char * outputFilename = argv[ 2 ];	// �o�̓t�@�C����
	int KindOfPointCloud;						// �_�Q�t�@�C���^�C�v �� 1:PCD( PointXYZRGB ), 2:PCD( PointXYZ ), 3:PLY( PointXYZRGB ), 4:PLY( PointXYZ )
	float VoxelGridSize;						// �{�N�Z���O���b�h�T�C�Y[�P��:m]
	float boxelGridSizeX = 10.00f;				// ����{�N�Z���O���b�h�T�C�Y X
	float boxelGridSizeY = 10.00f;				// ����{�N�Z���O���b�h�T�C�Y Y
	float boxelGridSizeZ = 10.00f;				// ����{�N�Z���O���b�h�T�C�Y Z

	/* �������̊m�� */
	pcl::PointCloud< pcl::PointXYZRGB >::Ptr inputCloud1( new pcl::PointCloud< pcl::PointXYZRGB > );	// PointXYZRGB �^���w��
	pcl::PointCloud< pcl::PointXYZ >::Ptr inputCloud2( new pcl::PointCloud< pcl::PointXYZ > );			// PointXYZ �^���w��
	pcl::PointCloud< pcl::PointXYZRGB >::Ptr outputCloud1( new pcl::PointCloud< pcl::PointXYZRGB > );	// PointXYZRGB �^���w��
	pcl::PointCloud< pcl::PointXYZ >::Ptr outputCloud2( new pcl::PointCloud< pcl::PointXYZ > );			// PointXYZ �^���w��

	/* �e��v���p�e�B�̎擾 */
	KindOfPointCloud = atoi( argv[ 3 ] );	// �_�Q�t�@�C���^�C�v �� 1:PCD( PointXYZRGB ), 2:PCD( PointXYZ ), 3:PLY( PointXYZRGB ), 4:PLY( PointXYZ )
	if( argc > 3 ){
		VoxelGridSize = atof( argv[ 4 ] );	// �{�N�Z���O���b�h�T�C�Y[�P��:m]
		boxelGridSizeX = VoxelGridSize;		// �{�N�Z���O���b�h�T�C�Y X
		boxelGridSizeY = VoxelGridSize;		// �{�N�Z���O���b�h�T�C�Y Y
		boxelGridSizeZ = VoxelGridSize;		// �{�N�Z���O���b�h�T�C�Y Z
	}
	

	/*** �_�Q�̓��� ***/
	if( KindOfPointCloud == 1 ) loadPCDPointCloudRGB( inputCloud1, inputFilename );	// ���͓_�Q�� PCD ���� PointXYZRGB �̏ꍇ
	if( KindOfPointCloud == 2 ) loadPCDPointCloud( inputCloud2, inputFilename );	// ���͓_�Q�� PCD ���� PointXYZ �̏ꍇ
	if( KindOfPointCloud == 3 ) loadPLYPointCloudRGB( inputCloud1, inputFilename );	// ���͓_�Q�� PLY ���� PointXYZRGB �̏ꍇ
	if( KindOfPointCloud == 4 ) loadPLYPointCloud( inputCloud2, inputFilename );	// ���͓_�Q�� PLY ���� PointXYZ �̏ꍇ


	/*** �_�Q���̃R�}���h���C���o�� ***/
	if( KindOfPointCloud == 1 || KindOfPointCloud == 3 ) cout << "�팸�O�_�Q��: " << inputCloud1->size() << " ��" << endl;
	if( KindOfPointCloud == 2 || KindOfPointCloud == 4 ) cout << "�팸�O�_�Q��: " << inputCloud2->size() << " ��" << endl;


	/*** �_�Q���̍팸 ***/
	if( KindOfPointCloud == 1 || KindOfPointCloud == 3 ) reductPointCloudRGB( inputCloud1, outputCloud1, boxelGridSizeX, boxelGridSizeY, boxelGridSizeZ );
	if( KindOfPointCloud == 2 || KindOfPointCloud == 4 ) reductPointCloud( inputCloud2, outputCloud2, boxelGridSizeX, boxelGridSizeY, boxelGridSizeZ );


	/*** �_�Q���̃R�}���h���C���o�� ***/
	if( KindOfPointCloud == 1 || KindOfPointCloud == 3 ) cout << "�팸��_�Q��: " << outputCloud1->size() << " ��" << endl;
	if( KindOfPointCloud == 2 || KindOfPointCloud == 4 ) cout << "�팸��_�Q��: " << outputCloud2->size() << " ��" << endl;


	/*** �_�Q�̏o�� ***/
	if( KindOfPointCloud == 1 ) savePointCloudRGBtoPCD( outputCloud1, outputFilename );
	if( KindOfPointCloud == 2 ) savePointCloudtoPCD( outputCloud2, outputFilename );
	if( KindOfPointCloud == 3 ) savePointCloudRGBtoPLY( outputCloud1, outputFilename );;
	if( KindOfPointCloud == 4 ) savePointCloudtoPLY( outputCloud2, outputFilename );


	/*** �v���O�����I���̃R�[�� ***/
	cout << "----- VoxelGrid �ɂ��_�Q���̍팸�I�� -----" << endl;


	return 0;
}