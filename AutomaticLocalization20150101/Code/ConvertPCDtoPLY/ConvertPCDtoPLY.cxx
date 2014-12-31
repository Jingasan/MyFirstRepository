/***** PCD �f�[�^�� PLY �f�[�^�ւ̕ϊ� *****/

//�y �R�}���h���C������ �z
// 1. ���͓_�Q�t�@�C����
// �� PCD �f�[�^
// 2. �o�͓_�Q�t�@�C����
// �� PLY �f�[�^
// 3. �_�Q�t�@�C���^�C�v
// �� 1:PCD( PointXYZRGB ), 2:PCD( PointXYZ )

// ��F
// Output/EdgePointData/ID_0002_LiverV31U10L1R10.pcd Output/EdgePointData/ID_0002_LiverV31U10L1R10.ply 1


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
	cout << "----- Convert PCDData to PLYData Start -----" << endl;


	/* �R�}���h���C�������ǂݍ��ݎ��s�� */
	if( argc < 3 ){
		cerr << "Caution! : �R�}���h���C�������G���[" << endl;
		cerr << "Usage: " << endl;
		cerr << argv[ 0 ] << endl << "1. inputFileName" << endl
			<< "2. outputFileName" << endl
			<< "3. KindOfPointCloud �� 1:PointXYZRGB, 2:PointXYZ" << endl;
		return EXIT_FAILURE;
	}


	/* �ϐ��̐錾 */
	const char * inputFilename = argv[ 1 ];		// ���̓t�@�C����
	const char * outputFilename = argv[ 2 ];	// �o�̓t�@�C����
	int KindOfPointCloud;						// �_�Q�t�@�C���^�C�v �� 1:PCD( PointXYZRGB ), 2:PCD( PointXYZ )

	/* �������̊m�� */
	pcl::PointCloud< pcl::PointXYZRGB >::Ptr inputCloud1( new pcl::PointCloud< pcl::PointXYZRGB > );	// PointXYZRGB �^���w��
	pcl::PointCloud< pcl::PointXYZ >::Ptr inputCloud2( new pcl::PointCloud< pcl::PointXYZ > );			// PointXYZ �^���w��

	/* �e��v���p�e�B�̎擾 */
	KindOfPointCloud = atoi( argv[ 3 ] );	// �_�Q�t�@�C���^�C�v �� 1:PCD( PointXYZRGB ), 2:PCD( PointXYZ )
	

	/*** �_�Q�̓��� ***/
	if( KindOfPointCloud == 1 ) loadPCDPointCloudRGB( inputCloud1, inputFilename );	// ���͓_�Q�� PointXYZRGB �̏ꍇ
	if( KindOfPointCloud == 2 ) loadPCDPointCloud( inputCloud2, inputFilename );	// ���͓_�Q�� PointXYZ �̏ꍇ


	/*** �_�Q���̃R�}���h���C���o�� ***/
	if( KindOfPointCloud == 1 ) cout << "�_�Q��: " << inputCloud1->size() << " ��" << endl;
	if( KindOfPointCloud == 2 ) cout << "�_�Q��: " << inputCloud2->size() << " ��" << endl;


	/*** �_�Q�̏o�� ***/
	if( KindOfPointCloud == 1 ) savePointCloudRGBtoPLY( inputCloud1, outputFilename );	// �o�͓_�Q�� PointXYZRGB �̏ꍇ
	if( KindOfPointCloud == 2 ) savePointCloudtoPLY( inputCloud2, outputFilename );	// �o�͓_�Q�� PointXYZ �̏ꍇ


	/*** �v���O�����I���̃R�[�� ***/
	cout << "----- Convert PCDData to PLYData End -----" << endl;


	return 0;
}