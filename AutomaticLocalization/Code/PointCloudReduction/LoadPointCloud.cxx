/***** 3�����_�Q�̓ǂݍ��� *****/


/*** �C���N���[�h�t�@�C�� ***/

/* PCL 1.6.0 */
#include <pcl/point_types.h>		// �_�Q�̃f�[�^�^
#include <pcl/io/pcd_io.h>			// PCD �f�[�^���o��
#include <pcl/io/ply_io.h>			// PLY �f�[�^���o��
#include <pcl/common/io.h>			// ���o��

/* �쐬�w�b�_�t�@�C�� */
#include "FunctionDefinition.h"	// �֐��̒�`


/*** ���O��Ԃ̐錾 ***/

/* C++ */
using namespace std;


/*** PCD �̓ǂݍ���( PointXYZRGB ) ***/
int loadPCDPointCloudRGB( pcl::PointCloud< pcl::PointXYZRGB >::Ptr cloud, const char* inputFileName ){

	cout << ">>> PCD �f�[�^( PointXYZRGB )�̓Ǎ���" << "\r";
	if( pcl::io::loadPCDFile( inputFileName, *cloud ) == -1 ){ cout << "Caution!: PCD �f�[�^�̓ǂݍ��ݎ��s�B" << endl; return 1; }
	cout << ">>> PCD �f�[�^�̓ǂݍ��� OK!" << endl;

	return 0;
}


/*** PCD �_�Q�̓ǂݍ���( PointXYZ ) ***/
int loadPCDPointCloud( pcl::PointCloud< pcl::PointXYZ >::Ptr cloud, const char* inputFileName ){

	cout << ">>> PCD �f�[�^( PointXYZ )�̓Ǎ���" << "\r";
	if( pcl::io::loadPCDFile( inputFileName, *cloud ) == -1 ){ cout << "Caution!: PCD �f�[�^�̓ǂݍ��ݎ��s�B" << endl; return 1; }
	cout << ">>> PCD �f�[�^�̓ǂݍ��� OK!" << endl;

	return 0;
}


/*** PLY �_�Q�̓ǂݍ���( PointXYZRGB ) ***/
int loadPLYPointCloudRGB( pcl::PointCloud< pcl::PointXYZRGB >::Ptr cloud, const char* inputFileName ){

	cout << ">>> PLY �f�[�^( PointXYZRGB )�̓Ǎ���" << "\r";
	if( pcl::io::loadPLYFile( inputFileName, *cloud ) == -1 ){ cout << "Caution!: PCD �f�[�^�̓ǂݍ��ݎ��s�B" << endl; return 1; }
	cout << ">>> PLY �f�[�^�̓ǂݍ��� OK!" << endl;

	return 0;
}


/*** PLY �_�Q�̓ǂݍ���( PointXYZ ) ***/
int loadPLYPointCloud( pcl::PointCloud< pcl::PointXYZ >::Ptr cloud, const char* inputFileName ){

	cout << ">>> PLY �f�[�^( PointXYZ )�̓Ǎ���" << "\r";
	if( pcl::io::loadPLYFile( inputFileName, *cloud ) == -1 ){ cout << "Caution!: PCD �f�[�^�̓ǂݍ��ݎ��s�B" << endl; return 1; }
	cout << ">>> PLY �f�[�^�̓ǂݍ��� OK!" << endl;

	return 0;
}

