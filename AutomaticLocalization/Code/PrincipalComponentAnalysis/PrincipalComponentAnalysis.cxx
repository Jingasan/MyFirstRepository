/***** �听������( PCA ) *****/


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

/* �w�b�_�t�@�C�� */
#include "Configuration.h"
#include "FunctionDefinition.h"


/*** ���O��Ԃ̐錾 ***/

/* C++ */
using namespace std;


/*** ���C�������֐� ***/
int main( void ){


	/* �����J�n�̃R�[�� */
	cout << "�y �v���O�����J�n �z" << endl;
	if( INPUTPOINTCLOUDNUM > POINTCLOUDNUM ){ cout << ">> Caution!: INPUTPOINTCLOUDNUM > POINTCLOUDNUM" << endl; return 1; }

	/* �ϐ��̐錾 */
	int maxPointCloudNum = 0;		// �ő�_�Q��
	int minPointCloudNum = INT_MAX;	// �ŏ��_�Q��
	int maxPointCloudID = -1;		// �ő�_�Q�����Ƃ�_�Q�ԍ�
	int minPointCloudID = -1;		// �ŏ��_�Q�����Ƃ�_�Q�ԍ�
	pcl::PointXYZRGB temp;
	pcl::PointCloud< pcl::PointXYZRGB >::Ptr *cloudCluster1[ INPUTPOINTCLOUDNUM ];
	for( int i = 0; i < INPUTPOINTCLOUDNUM; i++ ) cloudCluster1[ i ] = new pcl::PointCloud< pcl::PointXYZRGB >::Ptr( new pcl::PointCloud< pcl::PointXYZRGB > );

	/* �_�Q�f�[�^�̓ǂݍ���, �ő�_�Q���A�ŏ��_�Q���̎擾 */
	cout << ">>> �_�Q�f�[�^( PointXYZ )�̓Ǎ���..." << "\r";
	for( int i = 0; i < INPUTPOINTCLOUDNUM; i++ ){
		if( pcl::io::loadPCDFile( InputFileName[ i ], **cloudCluster1[ i ] ) == -1 ){ cout << ">>> �_�Q " << i << " �ǂݍ��ݎ��s" << endl; return 1; }
		if( cloudCluster1[ i ]->get()->size() > maxPointCloudNum ){ maxPointCloudNum = cloudCluster1[ i ]->get()->size(); maxPointCloudID = i; }
		if( cloudCluster1[ i ]->get()->size() < minPointCloudNum ){ minPointCloudNum = cloudCluster1[ i ]->get()->size(); minPointCloudID = i; }
	}
	cout << ">>> �_�Q�f�[�^( PointXYZ )�̓ǂݍ��� OK" << endl;

#ifdef COMMENT
	for( int i = 0; i < INPUTPOINTCLOUDNUM; i++ ) cout << i << ". �ő�_�Q���F" << maxPointCloudNum << "( ID�F" << maxPointCloudID << " )" << ", �ŏ��_�Q���F" << minPointCloudNum << "( ID�F" << minPointCloudID << " )" << endl;
#endif


	/*** �_�Q���̐��K�� ***/
	cout << ">>> ���f���_�Q���̐��K����..." << "\r";
	pcl::PointCloud< pcl::PointXYZRGB >::Ptr *cloudCluster2[ INPUTPOINTCLOUDNUM ];
	for( int i = 0; i < INPUTPOINTCLOUDNUM; i++ ) cloudCluster2[ i ] = new pcl::PointCloud< pcl::PointXYZRGB >::Ptr( new pcl::PointCloud< pcl::PointXYZRGB > );

	/* �e���f���_�Q�f�[�^�̓_�Q�����ŏ��_�Q���ɑ����� */
	for( int i = 0; i < INPUTPOINTCLOUDNUM; i++ ){
		for( int j = 0; j < minPointCloudNum; j++ ){
			
			temp.x = cloudCluster1[ i ]->get()->points[ j ].x;
			temp.y = cloudCluster1[ i ]->get()->points[ j ].y;
			temp.z = cloudCluster1[ i ]->get()->points[ j ].z;
			if( ( i % 15 ) == 0 ) temp.r = 255; temp.g = 255; temp.b = 255;
			if( ( i % 15 ) == 1 ) temp.r = 255; temp.g = 0; temp.b = 0;
			if( ( i % 15 ) == 2 ) temp.r = 0; temp.g = 255; temp.b = 0;
			if( ( i % 15 ) == 3 ) temp.r = 0; temp.g = 0; temp.b = 255;
			if( ( i % 15 ) == 4 ) temp.r = 255; temp.g = 255; temp.b = 0;
			if( ( i % 15 ) == 5 ) temp.r = 0; temp.g = 255; temp.b = 255;
			if( ( i % 15 ) == 6 ) temp.r = 255; temp.g = 0; temp.b = 255;
			if( ( i % 15 ) == 7 ) temp.r = 128; temp.g = 128; temp.b = 128;
			if( ( i % 15 ) == 8 ) temp.r = 128; temp.g = 0; temp.b = 0;
			if( ( i % 15 ) == 9 ) temp.r = 0; temp.g = 128; temp.b = 0;
			if( ( i % 15 ) == 10 ) temp.r = 0; temp.g = 0; temp.b = 128;
			if( ( i % 15 ) == 11 ) temp.r = 128; temp.g = 128; temp.b = 0;
			if( ( i % 15 ) == 12 ) temp.r = 0; temp.g = 128; temp.b = 128;
			if( ( i % 15 ) == 13 ) temp.r = 128; temp.g = 0; temp.b = 128;
			if( ( i % 15 ) == 14 ) temp.r = 0; temp.g = 0; temp.b = 0;
			cloudCluster2[ i ]->get()->points.push_back( temp );
		}
		cloudCluster2[ i ]->get()->width = minPointCloudNum;
		cloudCluster2[ i ]->get()->height = 1;
		cout << "ID�F" << i << ", �_�Q���F" << cloudCluster2[ i ]->get()->size() << endl;
	}
	cout << ">>> ���f���_�Q���̐��K�� OK" << endl;


#ifdef MERGE

	/* �_�Q�̓��� */
	pcl::PointCloud< pcl::PointXYZRGB >::Ptr *mergedCloud[ INPUTPOINTCLOUDNUM - 1 ];
	for( int i = 0; i < INPUTPOINTCLOUDNUM - 1; i++ ) mergedCloud[ i ] = new pcl::PointCloud< pcl::PointXYZRGB >::Ptr( new pcl::PointCloud< pcl::PointXYZRGB > );
	for( int i = 0; i < INPUTPOINTCLOUDNUM - 1; i++ ){
		if( i == 0 ) **mergedCloud[ i ] = **cloudCluster2[ i ] + **cloudCluster2[ i + 1 ];
		else **mergedCloud[ i ] = **mergedCloud[ i - 1 ] + **cloudCluster2[ i + 1 ];
	}

	/* �_�Q�̏o�͕ۑ� */
	savePointCloudRGBtoPLY( *mergedCloud[ INPUTPOINTCLOUDNUM - 2 ], OutputPLYFileName1 );

#endif
#ifdef VIEWER

	/* �_�Q�̕\�� */
	cout << ">>> �_�Q�f�[�^( PointXYZRGB )�̕\����" << "\r";
	for( int i = 0; i < INPUTPOINTCLOUDNUM; i++ ) showPointCloudRGB( *cloudCluster2[ i ], i );
	showPointCloudRGB( *mergedCloud[ INPUTPOINTCLOUDNUM - 2 ], INPUTPOINTCLOUDNUM - 2 );
	cout << ">>> �_�Q�f�[�^( PointXYZRGB )�̕\���I��" << endl;

#endif


	/*** �听������ ***/

	/* �f�U�C���s��̍쐬 */
	
	
	const int rowSize = 300;//3 * minPointCloudNum;	// �s�̗v�f�� �c ���f���_�Q���W 3 * N �� �� x1, y1, z1, x2, y2, z2, �c, xN, yN, zN
	const int colSize = INPUTPOINTCLOUDNUM;			// ��̗v�f�� �c ���f���� M �� �� ���f��1, ���f��2, ���f��3, �c, ���f��M
	//Eigen::MatrixXd designMatrix;
	//Eigen::Matrix< double, rowSize, colSize > designMatrix;





	/* �����I���̃R�[�� */
	cout << "�y �v���O�����I�� �z" << endl;

	return 0;
}