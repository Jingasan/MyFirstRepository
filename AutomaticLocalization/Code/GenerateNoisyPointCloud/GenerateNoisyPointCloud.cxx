/***** ���͓_�Q�ɑ΂��郉���_���m�C�Y�̎U�z *****/

// �y �R�}���h���C������ �z
// �w�肷��R�}���h���C������
// 1. ���͓_�Q�t�@�C����1.pcd
// 2. ���͓_�Q�t�@�C����2.pcd
// 3. �o�͓_�Q�t�@�C����3.pcd
// [ 4. �U�z���郉���_���m�C�Y�_�Q�� ]

// �� 1�F
// Input/PointData/ModelData/LiverR20/0001.pcd Output/NoisyPointData/ModelData/LiverR20/0001N200.pcd Output/NoisyPointData/ModelData/LiverR20/0001N200.ply 200

// �� 2�F
// Output/PlusGaussianNoisePointData/LiverR20/0001_1Gaussian.pcd Output/NoisyPointData/ModelData/LiverR20/0001_1GaussianN200.pcd Output/NoisyPointData/ModelData/LiverR20/0001_1GaussianN200.ply 200


/*** �C���N���[�h�t�@�C�� ***/

/* C */
#include <float.h>

/* C++ */
#include <iostream>
#include <fstream>
#include <vector>

/* PCL 1.6.0 */
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/common/io.h>
#include <pcl/point_types.h>

/* �ݒ�p�w�b�_�t�@�C�� */
#include "Configuration.h"
#include "FunctionDefinition.h"


/*** ���O��Ԃ̐錾 ***/

/* C++ */
using namespace std;


/*** ���C�������� ***/
int main( int argc, char *argv[] ){
	

	/* �R�}���h���C�������ǂݍ��ݎ��s�� */
	if( argc < 4 ){
		cerr << "Caution! : �R�}���h���C�������G���[" << endl;
		cerr << "Usage: " << endl;
		cerr << argv[ 0 ] << endl << " inputPointCloud.pcd outputPointCloud.pcd outputPointCloud.ply [ noiseNum ]" << endl;
		return EXIT_FAILURE;
	}


	/*** �ϐ��̐錾 ***/
	int noiseNum = NOISE_NUM;
	const char* inputFileName = argv[ 1 ];		// ���͓_�Q�t�@�C����
	const char* outputFileName1 = argv[ 2 ];	// �o�͓_�Q�t�@�C����
	const char* outputFileName2 = argv[ 3 ];	// �o�͓_�Q�t�@�C����
	if( argc > 4 ) noiseNum = atoi( argv[ 4 ] );
	double maxX = - DBL_MAX;	// ���͓_�Q���W�� X �ő�l
	double maxY = - DBL_MAX;	// ���͓_�Q���W�� Y �ő�l
	double maxZ = - DBL_MAX;	// ���͓_�Q���W�� Z �ő�l
	double minX = DBL_MAX;		// ���͓_�Q���W�� X �ŏ��l
	double minY = DBL_MAX;		// ���͓_�Q���W�� Y �ŏ��l
	double minZ = DBL_MAX;		// ���͓_�Q���W�� Z �ŏ��l
	double widthX;					// ���͓_�Q�� X ������
	double widthY;					// ���͓_�Q�� Y ������
	double widthZ;					// ���͓_�Q�� Z ������


	/*** �����_���m�C�Y�U�z�Ώۓ_�Q�̓ǂݍ��� ***/

	/* ���͓_�Q�̃������m�� */
	cout << ">>> ���͓_�Q( PointXYZRGB )�̓Ǎ���..." << "\r";
	pcl::PointCloud< pcl::PointXYZRGB >::Ptr cloud1( new pcl::PointCloud< pcl::PointXYZRGB > );
	if( pcl::io::loadPCDFile( inputFileName, *cloud1 ) == -1 ){ cout << ">>> ���͓_�Q�f�[�^�̓ǂݍ��ݎ��s" << endl; return 1; }
	cout << ">>> ���͓_�Q( PointXYZRGB )�̓ǂݍ��� OK" << endl;

	/* ���͓_�Q�̒[�_�̎擾 */
	for( int i = 0; i < cloud1->points.size(); i++ ){

		/* X, Y, Z �̍ŏ��l��ݒ� */
		if( cloud1->points[ i ].x < minX ) minX = cloud1->points[ i ].x;
		if( cloud1->points[ i ].y < minY ) minY = cloud1->points[ i ].y;
		if( cloud1->points[ i ].z < minZ ) minZ = cloud1->points[ i ].z;
		
		/* X, Y, Z �̍ő�l��ݒ� */
		if( cloud1->points[ i ].x > maxX ) maxX = cloud1->points[ i ].x;
		if( cloud1->points[ i ].y > maxY ) maxY = cloud1->points[ i ].y;
		if( cloud1->points[ i ].z > maxZ ) maxZ = cloud1->points[ i ].z;
	}

	/* ���͓_�Q���̎擾 */
	widthX = maxX - minX;	// �_�Q�� X ������
	widthY = maxY - minY;	// �_�Q�� Y ������
	widthZ = maxZ - minZ;	// �_�Q�� Z ������


	/*** �m�C�Y�_�Q�̍쐬 ***/

	/* �m�C�Y�_�Q�̃������m�� */
	pcl::PointCloud< pcl::PointXYZRGB >::Ptr noise1( new pcl::PointCloud< pcl::PointXYZRGB > );
	pcl::PointXYZRGB temp;

	/* �����_���m�C�Y�_�Q�̍쐬 */
	cout << "�m�C�Y�_�Q���F" << noiseNum << endl;
	for( int i = 0; i < noiseNum; i++ ){

		/* �����_���m�C�Y�_���W�F���͓_�Q�̑��ݍ��W�͈͓��Ɏ��܂�悤�ɍ쐬 */
		temp.x = minX + ( rand() % (int)widthX );	// X
		temp.y = minY + ( rand() % (int)widthY );	// Y
		temp.z = minZ + ( rand() % (int)widthZ );	// Z

		/* �����_���m�C�Y�_�F�F�� */
		temp.r = 255;
		temp.g = 255;
		temp.b = 255;
		noise1->points.push_back( temp );
	}
	noise1->width = noise1->points.size();
	noise1->height = 1;
	

	/*** ���͓_�Q�ɑ΂��郉���_���m�C�Y�_�Q�̎U�z ***/

	/* �����_���m�C�Y�U�z��̓��͓_�Q */
	pcl::PointCloud< pcl::PointXYZRGB >::Ptr noisyCloud1( new pcl::PointCloud< pcl::PointXYZRGB > );

	/* ���͓_�Q�ƃ����_���m�C�Y�_�Q�̓��� */
	*noisyCloud1 = *cloud1 + *noise1;


#ifdef VIEWER
	
	/*** �_�Q�̉��� ***/
	showPointCloudRGB( noisyCloud1, noiseNum );

#endif

	/*** �_�Q�̏o�͕ۑ� ***/
	savePointCloudRGBtoPCD( noisyCloud1, outputFileName1 );
	savePointCloudRGBtoPLY( noisyCloud1, outputFileName2 );

	return 0;
}
