/***** �_�Q�� R, t �ϊ� *****/
// ���͓_�Q��_�Q�̏d�S�𒆐S�Ɏw�肵�� R, t ������]���i

//�y �R�}���h���C������ �z
// ��1�F
// Input/PointData/ModelData/LiverR20/0001.pcd Output/TransformPointCloud/LiverR20/Ra30Rb30Rc30Tx0Ty0Tz0/0001.pcd Output/TransformPointCloud/LiverR20/Ra30Rb30Rc30Tx0Ty0Tz0/0001.ply Output/TransformPointCloud/LiverR20/Ra30Rb30Rc30Tx0Ty0Tz0/Ra30Rb30Rc30Tx0Ty0Tz0.csv 30.0 30.0 30.0 0 0 0
// ��2�F
// Input/PointData/ModelData/LiverR20/0001.pcd Output/TransformPointCloud/LiverR20/Ra100Rb100Rc30Tx0Ty0Tz0/0001.pcd Output/TransformPointCloud/LiverR20/Ra100Rb100Rc30Tx0Ty0Tz0/0001.ply Output/TransformPointCloud/LiverR20/Ra100Rb100Rc30Tx0Ty0Tz0/Ra100Rb100Rc30Tx0Ty0Tz0.csv 100.0 100.0 30.0 0 0 0


/*** �C���N���[�h�t�@�C�� ***/

/* C++ */
#include <iostream>
#include <vector>
#include <fstream>
#include <string>

/* C */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <limits.h>

/* OpenMP */
#include <omp.h>

/* PCL 1.6.0 */
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/common/io.h>
#include <pcl/point_types.h>
//#include <pcl/point_cloud.h>
#include <pcl/registration/transformation_estimation.h>

/* �w�b�_�t�@�C�� */
#include "Configuration.h"
#include "FunctionDefinition.h"


/*** ���O��Ԃ̐錾 ***/

/* C++ */
using namespace std;


/*** ���C�������֐� ***/
int main( int argc, char* argv[] ){


	/* �����J�n�̃R�[�� */
	cout << "�y �v���O�����J�n �z" << endl;


	/* �R�}���h���C�������ǂݍ��ݎ��s�� */
	if( argc < 11 ){
		cerr << "Caution! : �R�}���h���C�������G���[" << endl;
		cerr << "Usage: " << endl;
		cerr << argv[ 0 ] << endl;
		cerr << " 1. InputPcdFile"<< endl;
		cerr << " 2. OutputPcdFile" << endl;
		cerr << " 3. OutputPlyFile" << endl;
		cerr << " 4. OutputRtFile" << endl;
		cerr << " 5. ��degree" << endl;
		cerr << " 6. ��degree" << endl;
		cerr << " 7. ��degree" << endl;
		cerr << " 8. tx" << endl;
		cerr << " 9. ty" << endl;
		cerr << "10. tz" << endl;
		system( "pause" );
		return -1;
	}

	/* �t�@�C���p�X�̐ݒ� */
	const char *inputFileName1  = argv[ 1 ];	// ���͓_�Q�f�[�^��
	const char *outputFileName1 = argv[ 2 ];	// �o�͓_�Q�f�[�^��
	const char *outputFileName2 = argv[ 3 ];	// �o�͓_�Q�f�[�^��
	const char *outputFileName3 = argv[ 4 ];	// �쐬������]�s��C���i�x�N�g���̒l���L�q�����G�N�Z���t�@�C���܂ł̃p�X

	/* �ϐ��̐錾 */
	float alphaDegree = atof( argv[ 5 ] );				// ����
	float betaDegree = atof( argv[ 6 ] );				// ����
	float gammaDegree = atof( argv[ 7 ] );				// ����
	float tx = atof( argv[ 8 ] );						// tx
	float ty = atof( argv[ 9 ] );						// ty
	float tz = atof( argv[ 10 ] );						// tz
	float alpha = ( alphaDegree / 180 ) * PAI;			// �����̃��W�A���l
	float beta = ( betaDegree / 180 ) * PAI;			// �����̃��W�A���l
	float gamma = ( gammaDegree / 180 ) * PAI;			// �����̃��W�A���l
	Eigen::Matrix3f alphaR;								// ���ɂ������]�s��
	Eigen::Matrix3f betaR;								// ���ɂ������]�s��
	Eigen::Matrix3f gammaR;								// ���ɂ������]�s��
	Eigen::Matrix3f R;									// ��]�s�� R
	Eigen::Vector3f t;									// ���i�x�N�g�� t
	Eigen::Matrix4f Rt;									// �A�t�B���ϊ��s�� Rt
	pcl::PointXYZRGB temp;								// �_���W�̈ꎞ�i�[�p�ϐ�
	

	/* �_�Q�����i�[����\���̕ϐ��̒�` */
	PointCloudInfo modelInformation;	// ���f���_�Q
	


	/*** �_�Q�̓ǂݍ��݁FPCD �f�[�^ ***/
	cout << "�y �_�Q�f�[�^�̓ǂݍ��� �z" << endl;
	
	/* R, t �ϊ��Ώۓ_�Q�f�[�^�̓ǂݍ��� */
	cout << ">>> R, t �ϊ��Ώۓ_�Q( PointXYZ )�̓Ǎ���..." << "\r";
	pcl::PointCloud< pcl::PointXYZ >::Ptr cloudM( new pcl::PointCloud< pcl::PointXYZ > );	// ���͓_�Q( PointXYZ )�̃������m��
	if( pcl::io::loadPCDFile( inputFileName1, *cloudM ) == -1 ){ cout << ">>> R, t �ϊ��Ώۓ_�Q�f�[�^�̓ǂݍ��ݎ��s" << endl; system( "pause" ); return -1; }
	cout << ">>> R, t �ϊ��Ώۓ_�Q( PointXYZ )�̓ǂݍ��� OK" << endl;



	/*** ���͓_�Q�̍��W�ϊ� ***/
	cout << "�y ���͓_�Q�̍��W�ϊ�(�d�S�����_���W�ɐݒ�) �z" << endl;

	/* ���W�ϊ���̓_�Q�̃������m�� */
	pcl::PointCloud< pcl::PointXYZRGB >::Ptr modelPointCloud( new pcl::PointCloud< pcl::PointXYZRGB > );		// �d�S�ϊ���̃��f���_�Q

	/* ������ */
	modelInformation.Xmin = DBL_MAX;
	modelInformation.Ymin = DBL_MAX;
	modelInformation.Zmin = DBL_MAX;
	modelInformation.Xmax = -DBL_MAX;
	modelInformation.Ymax = -DBL_MAX;
	modelInformation.Zmax = -DBL_MAX;

	/* ���f���_�Q�̒[�_�̎Z�o */
	for( int i = 0; i < cloudM->points.size(); i++ ){
		
		/* X, Y, Z �̍ŏ��l��ݒ� */
		if( cloudM->points[ i ].x < modelInformation.Xmin ) modelInformation.Xmin = cloudM->points[ i ].x;
		if( cloudM->points[ i ].y < modelInformation.Ymin ) modelInformation.Ymin = cloudM->points[ i ].y;
		if( cloudM->points[ i ].z < modelInformation.Zmin ) modelInformation.Zmin = cloudM->points[ i ].z;
		
		/* X, Y, Z �̍ő�l��ݒ� */
		if( cloudM->points[ i ].x > modelInformation.Xmax ) modelInformation.Xmax = cloudM->points[ i ].x;
		if( cloudM->points[ i ].y > modelInformation.Ymax ) modelInformation.Ymax = cloudM->points[ i ].y;
		if( cloudM->points[ i ].z > modelInformation.Zmax ) modelInformation.Zmax = cloudM->points[ i ].z;
	}

	/* �_�Q�̏d�S�����f�����Ƃ��Ċi�[ */
	modelInformation.gravX = ( modelInformation.Xmax + modelInformation.Xmin ) / 2;
	modelInformation.gravY = ( modelInformation.Ymax + modelInformation.Ymin ) / 2;
	modelInformation.gravZ = ( modelInformation.Zmax + modelInformation.Zmin ) / 2;

	/* �_�Q�̏d�S�����_�ɐݒ� */
	modelInformation.Xmin -= modelInformation.gravX; modelInformation.Xmax -= modelInformation.gravX;
	modelInformation.Ymin -= modelInformation.gravY; modelInformation.Ymax -= modelInformation.gravY;
	modelInformation.Zmin -= modelInformation.gravZ; modelInformation.Zmax -= modelInformation.gravZ;

	/* ���f���_�Q */
	for( int i = 0; i < cloudM->size(); i++ ){
			
		/* ���f���_�Q�� DF ���W�n�֕ϊ��FDF ���W�n�͓_�Q�̏d�S�����_ */
		temp.x = cloudM->points[ i ].x - modelInformation.gravX;
		temp.y = cloudM->points[ i ].y - modelInformation.gravY;
		temp.z = cloudM->points[ i ].z - modelInformation.gravZ;
		temp.r = 255;
		temp.g = 0;
		temp.b = 0;
		modelPointCloud->points.push_back( temp );
	}
	modelPointCloud->width = cloudM->points.size();
	modelPointCloud->height = 1;



	/*** 3�����̃A�t�B���ϊ��s��̍쐬 ***/
	cout << "�y 3�����̃A�t�B���ϊ��s��̍쐬 �z" << endl;
	cout << "�w��l�F" << endl;
	cout << "( ��, ��, ��, tx, ty, tz ) = ( " << alphaDegree << "��, " << betaDegree << "��, " << gammaDegree << "��, " << tx << ", " << ty << ", " << tz << " )" << endl;
	
	/* �I�C���[�p�ɂ���]�s�� R �̍쐬 */
	alphaR << cos( alpha ), -sin( alpha ), 0, sin( alpha ), cos( alpha ), 0, 0, 0, 1;
	betaR << cos( beta ), 0, sin( beta ), 0, 1, 0, -sin( beta ), 0, cos( beta );
	gammaR << cos( gamma ), -sin( gamma ), 0, sin( gamma ), cos( gamma ), 0, 0, 0, 1;
	R = alphaR * betaR * gammaR;

	/* ���i�x�N�g�� t �̍쐬 */
	t << tx, ty, tz;

	/* �A�t�B���ϊ��s�� Rt �̍쐬 */
	Rt << R( 0, 0 ), R( 0, 1 ), R( 0, 2 ), t( 0 ), R( 1, 0 ), R( 1, 1 ), R( 1, 2 ), t( 1 ), R( 2, 0 ), R( 2, 1 ), R( 2, 2 ), t( 2 ), 0, 0, 0, 1;

	/* �쐬���ꂽ�A�t�B���ϊ��s��̃R�}���h���C���\�� */
	cout << ">> Rt�F" << endl;
	cout << Rt << endl;
	cout << ">> |R|�F" << endl;
	cout << R.determinant() << endl;
	cout << ">> |Rt|�F" << endl;
	cout << Rt.determinant() << endl;



	/*** ���͓_�Q�̃A�t�B���ϊ� ***/
	cout << "�y ���͓_�Q�� R, t �ϊ� �z" << endl;

	/* �������̊m�� */
	pcl::PointCloud< pcl::PointXYZRGB >::Ptr transformedCloud( new pcl::PointCloud< pcl::PointXYZRGB > );	// ���W�ϊ����3�����_�Q( PointXYZ �^���w�� )
	
	/* 4 �~ 4 �� Rt �s��ɂ��A�t�B���ϊ� */
	pcl::transformPointCloud( *modelPointCloud, *transformedCloud, Rt );

	/* R, t �ϊ���̓_�Q�F�̕ύX */
	for( int i = 0; i < transformedCloud->size(); i++ ){
		transformedCloud->points[ i ].r = 0;
		transformedCloud->points[ i ].g = 255;
		transformedCloud->points[ i ].b = 0;
	}



	/*** 2�_�Q�̓����Ɖ��� ***/
#ifdef VIEWER
	cout << "�y �ϊ��O�̓_�Q�ƕϊ���̓_�Q�̉��� �z" << endl;
	cout << ">> R, t �ϊ��O�̓_�Q �F RED" << endl;
	cout << ">> R, t �ϊ���̓_�Q �F GREEN" << endl;

	/* �������̊m�� */
	pcl::PointCloud< pcl::PointXYZRGB >::Ptr merged( new pcl::PointCloud< pcl::PointXYZRGB > );	// ���W�ϊ����3�����_�Q( PointXYZ �^���w�� )
	
	/* ���� */
	*merged = *modelPointCloud + *transformedCloud;

	/* ���� */
	showPointCloudRGB( merged, 1 );
#endif


	/*** R, t �ϊ���̓_�Q�̓_�Q�����̍��W�n�ɖ߂� ***/
	cout << "�y R, t �ϊ���̓_�Q�̓_�Q�����̍��W�n�ɖ߂� �z" << endl;

	/* ���W�ϊ���̓_�Q�̃������m�� */
	pcl::PointCloud< pcl::PointXYZRGB >::Ptr finalCloud( new pcl::PointCloud< pcl::PointXYZRGB > );

	/* R, t �ϊ���̓_�Q�̓_�Q�����̍��W�n�ɖ߂� */
	for( int i = 0; i < transformedCloud->size(); i++ ){
			
		/* ���f���_�Q�� DF ���W�n�֕ϊ��FDF ���W�n�͓_�Q�̏d�S�����_ */
		temp.x = transformedCloud->points[ i ].x + modelInformation.gravX;
		temp.y = transformedCloud->points[ i ].y + modelInformation.gravY;
		temp.z = transformedCloud->points[ i ].z + modelInformation.gravZ;
		temp.r = transformedCloud->points[ i ].r;
		temp.g = transformedCloud->points[ i ].g;
		temp.b = transformedCloud->points[ i ].b;
		finalCloud->points.push_back( temp );
	}
	finalCloud->width = transformedCloud->points.size();
	finalCloud->height = 1;



	/*** �_�Q�̏o�͕ۑ� ***/
	cout << "�y �_�Q�̏o�͕ۑ� �z" << endl;
	savePointCloudRGBtoPCD( finalCloud, outputFileName1 );
	savePointCloudRGBtoPLY( finalCloud, outputFileName2 );


	/*** Rt �̃G�N�Z���o�͕ۑ� ***/
	cout << "�y Rt�̏o�͕ۑ� �z" << endl;

	/* �t�@�C���X�g���[���ϐ��̒�` */
	ofstream output;

	/* �o�̓t�@�C���p�X�̐ݒ� */
	output.open( outputFileName3 );

	/* �G�N�Z���o�� */
	output << Rt( 0, 0 ) << "," << Rt( 0, 1 ) << "," << Rt( 0, 2 ) << "," << Rt( 0, 3 ) << endl;
	output << Rt( 1, 0 ) << "," << Rt( 1, 1 ) << "," << Rt( 1, 2 ) << "," << Rt( 1, 3 ) << endl;
	output << Rt( 2, 0 ) << "," << Rt( 2, 1 ) << "," << Rt( 2, 2 ) << "," << Rt( 2, 3 ) << endl;
	output << Rt( 3, 0 ) << "," << Rt( 3, 1 ) << "," << Rt( 3, 2 ) << "," << Rt( 3, 3 ) << endl;

	/* �������̉�� */
	output.close();


	/* �����I���̃R�[�� */
	cout << "�y �v���O�����I�� �z" << endl;

	return 0;
}