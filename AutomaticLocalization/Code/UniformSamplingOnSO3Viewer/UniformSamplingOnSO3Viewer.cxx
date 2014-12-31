/***** ��]��Ԃ̋ϓ��ȃT���v�����O�̉��� *****/


/*** �t���O�̒�` ***/
//#define COMMENT	// �r�����ʂ̃R�}���h���C���o�� ( ON : ���s, OFF : ���� )


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
#include <CodeAnalysis\Warnings.h>

/* OpenMP */
#include <omp.h>

/* PCL 1.6.0 */
#pragma warning( push )
#pragma warning( disable : ALL_CODE_ANALYSIS_WARNINGS )
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/common/io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#pragma warning( pop )


/*** ���O��Ԃ̐錾 ***/

/* C++ */
using namespace std;


/*** �萔�̒�` ***/
char input3RadianFileName[ 64 ] = "Output/SamplingR/36864radian.csv";
char outputSamplingS2HopfPCDFileName[ 64 ] = "Output/SamplingR/samplingS2Hopf.pcd";
char outputSamplingS2HopfPLYFileName[ 64 ] = "Output/SamplingR/samplingS2Hopf.ply";
char outputSamplingS2HEALPixPCDFileName[ 64 ] = "Output/SamplingR/samplingS2HEALPix.pcd";
char outputSamplingS2HEALPixPLYFileName[ 64 ] = "Output/SamplingR/samplingS2HEALPix.ply";


/*** ���C�������֐� ***/
int main( void ){


	/* �����J�n�̃R�[�� */
	cout << "�y �v���O�����J�n �z" << endl;
	

	/* �ϐ��̒�` */
	FILE *inputFile;						// �t�@�C���^�ϐ�
	vector< vector< float > > radians;		// ��]�p ��, ��, �� ( �P�ʁF���W�A�� ) �̊i�[�p�s��
	vector< float > radian;					// ��]�p ��, ��, �� ( �P�ʁF���W�A�� ) �̊i�[�p�x�N�g��
	vector< vector< float > > radiansS2;	// ���ʍ��W S2 ��̉�]�p ��, �� ( �P�ʁF���W�A�� ) �̊i�[�p�s��
	vector< float > radianS2;				// ���ʍ��W S2 ��̉�]�p ��, �� ( �P�ʁF���W�A�� ) �̊i�[�p�x�N�g��
	int count = 0;	// �J�E���^�ϐ�
	

	/*** �G�N�Z���t�@�C���̓ǂݍ��� ***/

	/* �t�@�C���ǂݍ��ݎ��s�� */
	if( ( inputFile = fopen( input3RadianFileName, "r" ) ) == NULL ){
		cout << "�t�@�C�� " << input3RadianFileName << " ���J���܂���" << endl;
		exit( 0 );
	}
	
	/* �������̊m�� */
	radians.resize( 0 );
	radiansS2.resize( 0 );
	radian.resize( 3 );
	radianS2.resize( 2 );
	
	/* �t�@�C�����e�̓ǂݍ��� */
	while( fscanf( inputFile, "%f,%f,%f", &radian[ 0 ], &radian[ 1 ], &radian[ 2 ] ) != EOF ){

		/* 2�����z��ւ�1�����z��̒ǉ� */
		radians.push_back( radian );	// �x�N�g���̖����ɑ�1������ǉ�����
		if( ( count % 6 ) == 0 ){
			radianS2[ 0 ] = radian[ 0 ], radianS2[ 1 ] = radian[ 1 ];	// ��]�p�Ƃƃ��݂̂̎擾
			radiansS2.push_back( radianS2 );	// �x�N�g���̖����ɑ�1������ǉ�����
		}
		count++;
	}
	
	/* �������̉�� */
	fclose( inputFile );
	

#ifdef COMMENT

	/* ��]�p ��, ��, �� ( �P�ʁF���W�A�� ) �̃R�}���h���C���o�� */
	cout << "�y ��]�p ��, ��, �� ( �P�ʁF���W�A�� ) �z" << endl;
	for( int i = 0; i < radians.size(); i++ ){
		for( int j = 0; j < radians[ i ].size(); j++ ){
			cout << radians[ i ][ j ] << " ";
		}
		cout << endl;
	}

	/* ��]�p ��, �� ( �P�ʁF���W�A�� ) �̃R�}���h���C���o�� */
	cout << "�y ��]�p ��, �� ( �P�ʁF���W�A�� ) �z" << endl;
	for( int i = 0; i < radiansS2.size(); i++ ){
		for( int j = 0; j < radiansS2[ i ].size(); j++ ){
			cout << radiansS2[ i ][ j ] << " ";
		}
		cout << endl;
	}

#endif // COMMENT


	/*** ���ʍ��WS2���3�����_�Q�̍쐬 ( Hopf���W�\�� ) ***/
	cout << "�y ���ʍ��WS2���3�����_�Q�̉��� ( Hopf���W�\�� ) �z" << endl;

	/* 3�����_�Q�̍쐬 */
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr sampS2Hopf( new pcl::PointCloud<pcl::PointXYZRGB> );
	pcl::PointXYZRGB temp1;
	for( int i = 0; i < radiansS2.size(); i++ ){
		temp1.x = cos( radiansS2[ i ][ 0 ] / 2 );
		temp1.y = sin( radiansS2[ i ][ 0 ] / 2 ) * sin( radiansS2[ i ][ 1 ] );
		temp1.z = sin( radiansS2[ i ][ 0 ] / 2 ) * cos( radiansS2[ i ][ 1 ] );
		temp1.r = 255;
		temp1.g = 0;
		temp1.b = 255;
		sampS2Hopf->points.push_back( temp1 );
		//temp1.x = - cos( radiansS2[ i ][ 0 ] / 2 );
		//temp1.y = - sin( radiansS2[ i ][ 0 ] / 2 ) * sin( radiansS2[ i ][ 1 ] );
		//temp1.z = - sin( radiansS2[ i ][ 0 ] / 2 ) * cos( radiansS2[ i ][ 1 ] );
		//temp1.r = 0;
		//temp1.g = 255;
		//temp1.b = 255;
		//sampS2Hopf->points.push_back( temp1 );
	}
	sampS2Hopf->width = sampS2Hopf->points.size();
	sampS2Hopf->height = 1;


	/*** 3�����_�Q�� VIEWER �\�� ***/
	pcl::visualization::CloudViewer viewer1( "Simple Cloud Viewer 1" );	// Viewer �̖��O���u Simple Cloud Viewer �v�ɐݒ�
	viewer1.showCloud( sampS2Hopf );	// �_�Q�̕\��
	while( !viewer1.wasStopped() ){}	// Viewer ��������܂Ŗ������[�v 
	// [ Viewer �̑�����@ ]
	// �E���h���b�O - ���_�̉�]
	// �EShift + ���h���b�O - ���_�̕��s�ړ�
	// �ECtrl + ���h���b�O - ��ʏ�̉�]
	// �E�E�h���b�O - �Y�[��
	// �Eg�F���W���[�̕\��
	// �Ej�F�X�N���[���V���b�g�̕ۑ�


	/*** 3�����_�Q�̏o�͕ۑ� ***/
	
	/* �_�Q( PCD �f�[�^ )�̏������� */
	cout << " PCD �f�[�^( PointXYZRGB )�̏o�͕ۑ���..." << endl;
	if( pcl::io::savePCDFileASCII( outputSamplingS2HopfPCDFileName, *sampS2Hopf ) == -1 ){ cout << " PCD �f�[�^�̏������ݎ��s�B" << endl << " " << outputSamplingS2HopfPCDFileName << " ���o�͕ۑ��ł��܂���ł����B" << endl; return( -1 ); }
	cout << " PCD �f�[�^�̏o�͕ۑ� OK!" << endl;

	/* �_�Q( PLY �f�[�^ )�̏������� */
	cout << " PLY �f�[�^( PointXYZRGB )�̏o�͕ۑ���..." << endl;
	if( pcl::io::savePLYFileASCII( outputSamplingS2HopfPLYFileName, *sampS2Hopf ) == -1 ){ cout << " PLY �f�[�^�̏������ݎ��s�B" << endl << " " << outputSamplingS2HopfPLYFileName << " ���o�͕ۑ��ł��܂���ł����B" << endl; return( -1 ); }
	cout << " PLY �f�[�^�̏o�͕ۑ� OK!" << endl;


	/*** ���ʍ��WS2���3�����_�Q�̍쐬 ( �ʏ�̋��ʍ��W�\�� ) ***/
	cout << "�y ���ʍ��WS2���3�����_�Q�̉��� ( �ʏ�̋��ʍ��W�\�� ) �z" << endl;

	/* 3�����_�Q�̍쐬 */
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr sampS2HEALPix( new pcl::PointCloud<pcl::PointXYZRGB> );
	pcl::PointXYZRGB temp2;
	for( int i = 0; i < radiansS2.size(); i++ ){
		temp2.x = sin( radiansS2[ i ][ 0 ] ) * cos( radiansS2[ i ][ 1 ] );
		temp2.y = sin( radiansS2[ i ][ 0 ] ) * sin( radiansS2[ i ][ 1 ] );
		temp2.z = cos( radiansS2[ i ][ 0 ] );
		temp2.r = 0;
		temp2.g = 255;
		temp2.b = 0;
		sampS2HEALPix->points.push_back( temp2 );
	}
	sampS2HEALPix->width = sampS2HEALPix->points.size();
	sampS2HEALPix->height = 1;


	/*** 3�����_�Q�� VIEWER �\�� ***/
	pcl::visualization::CloudViewer viewer2( "Simple Cloud Viewer 2" );	// Viewer �̖��O���u Simple Cloud Viewer �v�ɐݒ�
	viewer2.showCloud( sampS2HEALPix );	// �_�Q�̕\��
	while( !viewer2.wasStopped() ){}	// Viewer ��������܂Ŗ������[�v 
	// [ Viewer �̑�����@ ]
	// �E���h���b�O - ���_�̉�]
	// �EShift + ���h���b�O - ���_�̕��s�ړ�
	// �ECtrl + ���h���b�O - ��ʏ�̉�]
	// �E�E�h���b�O - �Y�[��
	// �Eg�F���W���[�̕\��
	// �Ej�F�X�N���[���V���b�g�̕ۑ�


	/*** 3�����_�Q�̏o�͕ۑ� ***/
	
	/* �_�Q( PCD �f�[�^ )�̏������� */
	cout << " PCD �f�[�^( PointXYZRGB )�̏o�͕ۑ���..." << endl;
	if( pcl::io::savePCDFileASCII( outputSamplingS2HEALPixPCDFileName, *sampS2HEALPix ) == -1 ){ cout << " PCD �f�[�^�̏������ݎ��s�B" << endl << " " << outputSamplingS2HEALPixPCDFileName << " ���o�͕ۑ��ł��܂���ł����B" << endl; return( -1 ); }
	cout << " PCD �f�[�^�̏o�͕ۑ� OK!" << endl;

	/* �_�Q( PLY �f�[�^ )�̏������� */
	cout << " PLY �f�[�^( PointXYZRGB )�̏o�͕ۑ���..." << endl;
	if( pcl::io::savePLYFileASCII( outputSamplingS2HEALPixPLYFileName, *sampS2HEALPix ) == -1 ){ cout << " PLY �f�[�^�̏������ݎ��s�B" << endl << " " << outputSamplingS2HEALPixPLYFileName << " ���o�͕ۑ��ł��܂���ł����B" << endl; return( -1 ); }
	cout << " PLY �f�[�^�̏o�͕ۑ� OK!" << endl;


	/* �����I���̃R�[�� */
	cout << "�y �v���O�����I�� �z" << endl;


	return 0;
}






#ifdef CUT
	
	ifstream file( inputS2FileName );
	vector< vector< string > > values;
	string str;
	int p;
	
	if( file.fail() ){
		cerr << "failed." << endl;
		exit( 0 );
	}
	
    while( getline( file, str ) ){
		
		/* �R�����g�ӏ��͏��� */
		if( ( p = str.find( "//" ) ) != str.npos ) continue;
		vector< string > inner;
		
		/* �R���}�����邩��T���A�����܂ł�values�Ɋi�[ */
		while( ( p = str.find( "," ) ) != str.npos ){
			inner.push_back( str.substr( 0, p ) );
			
			//str�̒��g��", "��2�������΂�
			str = str.substr( p + 2 );
		}
		
		inner.push_back( str );
		values.push_back( inner );
	}

    for( unsigned int i = 0; i < values.size(); ++i){
        for( unsigned int j = 0; j < values[i].size(); ++j){
            cout << values[i][j] << ",";
        }
        cout << endl;
    }

#endif