/***** ���͓_�Q�ɑ΂���K�E�V�A���m�C�Y�̕t�� *****/

// �y �R�}���h���C������ �z
// �w�肷��R�}���h���C������
// 1. ���͓_�Q�t�@�C����1.pcd
// 2. ���͓_�Q�t�@�C����2.pcd
// 3. �o�͓_�Q�t�@�C����3.pcd
// [ 4. �K�E�V�A���̕W���΍����̃X�P�[���l ]

// ��F
// Input/PointData/ModelData/LiverR20/0001.pcd Output/PlusGaussianNoisePointData/LiverR20/0001_1Gaussian.pcd Output/PlusGaussianNoisePointData/LiverR20/0001_1Gaussian.ply 1


/*** �C���N���[�h�t�@�C�� ***/

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
	

	/* �v���O�����J�n */
	cout << "�y �v���O�����J�n �z" << endl;

	/* �R�}���h���C�������ǂݍ��ݎ��s�� */
	if( argc < 4 ){
		cerr << "Caution! : �R�}���h���C�������G���[" << endl;
		cerr << "Usage: " << endl;
		cerr << argv[ 0 ] << endl << " inputPointCloud.pcd outputPointCloud.pcd outputPointCloud.ply [ NoiseScale ]" << endl;
		return EXIT_FAILURE;
	}

	/* �ϐ��̐錾 */
	float noiseScale = NOISESCALE;					// �K�E�V�A���̕W���΍����̃X�P�[���l�̊���l
	const char* inputFileName = argv[ 1 ];			// ���͓_�Q�t�@�C����
	const char* outputFileName1 = argv[ 2 ];		// �o�͓_�Q�t�@�C����
	const char* outputFileName2 = argv[ 3 ];		// �o�͓_�Q�t�@�C����
	if( argc > 4 ) noiseScale = atof( argv[ 4 ] );	// �K�E�V�A���̕W���΍����̃X�P�[���l
	FILE *inputFp1;									// �K�E�V�A���m�C�Y�t�@�C��
	vector< float > gaussianNoiseXYZ;				// �K�E�V�A���m�C�Y�̒l�Fx, y, z
	int i = 0;										// �K�E�V�A���m�C�Y���̃J�E���^

	/* �������̊m�� */
	gaussianNoiseXYZ.resize( 3 );	// �z�񐔁F x, y, z �̃K�E�V�A���m�C�Y


	/*** �m�C�Y�t���Ώۓ_�Q�̓ǂݍ��� ***/

	/* ���͓_�Q�̃������m�� */
	cout << ">>> ���͓_�Q( PointXYZRGB )�̓Ǎ���..." << "\r";
	pcl::PointCloud< pcl::PointXYZRGB >::Ptr inputCloud( new pcl::PointCloud< pcl::PointXYZRGB > );
	if( pcl::io::loadPCDFile( inputFileName, *inputCloud ) == -1 ){ cout << ">>> ���͓_�Q�f�[�^�̓ǂݍ��ݎ��s" << endl; return 1; }
	cout << ">>> ���͓_�Q( PointXYZRGB )�̓ǂݍ��� OK" << endl;


	/*** �K�E�V�A���m�C�Y�t���_�Q�̍쐬 ***/

	/* �K�E�V�A���m�C�Y�t���_�Q�̃������m�� */
	pcl::PointCloud< pcl::PointXYZRGB >::Ptr gaussianNoisedCloud( new pcl::PointCloud< pcl::PointXYZRGB > );
	pcl::PointXYZRGB temp;

	/* �K�E�V�A���m�C�Y�t�@�C���̓W�J */
	if( ( inputFp1 = fopen( InputGaussianNoiseFileName, "r" ) ) == NULL ){ cout << "Caution!: " << InputGaussianNoiseFileName << " open error" << endl; return 1; }

	/* �t�@�C�����e�̓ǂݍ��݂Ɠ��͓_���W�ɑ΂���K�E�V�A���m�C�Y�̕t�� */
	cout << "----- �K�E�V�A���m�C�Y�̕t�� -----" << endl;
	cout << ">> The number of Input Points�F" << inputCloud->size() << endl;
	while( fscanf( inputFp1, "%f,%f,%f", &gaussianNoiseXYZ[ 0 ], &gaussianNoiseXYZ[ 1 ], &gaussianNoiseXYZ[ 2 ] ) != EOF ){

		temp.x = inputCloud->points[ i ].x + noiseScale * gaussianNoiseXYZ[ 0 ];
		temp.y = inputCloud->points[ i ].y + noiseScale * gaussianNoiseXYZ[ 1 ];
		temp.z = inputCloud->points[ i ].z + noiseScale * gaussianNoiseXYZ[ 2 ];
		temp.r = 255;
		temp.g = 255;
		temp.b = 0;
		gaussianNoisedCloud->points.push_back( temp );

#ifdef COMMENT
		if( i % 100 == 0 ){
			cout << ">> Point Number  �F" << i << endl;
			cout << ">> Gaussian Noise�F" << noiseScale * gaussianNoiseXYZ[ 0 ] << " " << noiseScale * gaussianNoiseXYZ[ 1 ] << " " << noiseScale * gaussianNoiseXYZ[ 2 ] << endl;
			cout << ">> Input Point   �F" << inputCloud->points[ i ].x << " " << inputCloud->points[ i ].y << " " << inputCloud->points[ i ].z << endl;
			cout << ">> Output Point  �F" << gaussianNoisedCloud->points[ i ].x << " " << gaussianNoisedCloud->points[ i ].y << " " << gaussianNoisedCloud->points[ i ].z << endl;
		}
#endif
		i++;
		if( i == inputCloud->size() ) break;	// ���͓_�Q���ׂĂɃK�E�V�A���m�C�Y���t�����ꂽ�ꍇ
	}
	gaussianNoisedCloud->width = gaussianNoisedCloud->points.size();
	gaussianNoisedCloud->height = 1;

	/* ���̓K�E�V�A���m�C�Y�̐����m�C�Y�t���Ώۂ̓_�Q���������Ȃ��ꍇ */
	if( i < inputCloud->size() ){
		cout << "Caution!�FThe number of Gaussian noise points is fewer than the input points." << endl;
		cout << "The number of Gaussian Noise�F" << i << endl;
		return 1;
	}

	/* �������̉�� */
	fclose( inputFp1 );


#ifdef VIEWER
	
	/*** �_�Q�̉��� ***/
	cout << ">> ���͓_�Q�\����" << "\r";
	showPointCloudRGB( inputCloud, inputCloud->size() );
	cout << ">> ���͓_�Q�\���I��" << endl;
	cout << ">> �K�E�V�A���m�C�Y�t���_�Q�\����" << "\r";
	showPointCloudRGB( gaussianNoisedCloud, i );
	cout << ">> �K�E�V�A���m�C�Y�t���_�Q�\���I��" << endl;

#endif

	/*** �_�Q�̏o�͕ۑ� ***/
	savePointCloudRGBtoPCD( gaussianNoisedCloud, outputFileName1 );
	savePointCloudRGBtoPLY( gaussianNoisedCloud, outputFileName2 );

	/* �v���O�����I�� */
	cout << "�y �v���O�����I�� �z" << endl;

	return 0;
}
