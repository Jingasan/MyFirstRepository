/***** CSV �f�[�^���� MHA �f�[�^�ւ̕ϊ� *****/

// �y �R�}���h���C������ �z
// 1. ���̓{�����[���f�[�^��.csv
// 2. �o�̓{�����[���f�[�^��.mha


/*** �C���N���[�h�t�@�C�� ***/

/* ITK 4.5.2 */
#include "itkImage.h"							// �摜�f�[�^�^
#include "itkImageFileReader.h"					// �摜�̏�������
#include "itkImageFileWriter.h"					// �摜�̓ǂݍ���
#include "itkCastImageFilter.h"					// �摜�̃f�[�^�^�̕ϊ�
#include "itkRescaleIntensityImageFilter.h"		// �s�N�Z���l�̃��X�P�[��
#include "itkCannyEdgeDetectionImageFilter.h"	// CannyEdgeDetection �t�B���^
#include "itkImageRegionIteratorWithIndex.h"	// Iterator

/* C++ */
#include <iostream>
#include <fstream>
#include <vector>

/* C */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>


/*** ���O��Ԃ̐錾 ***/
using namespace std;


/*** ���C�������� ***/
int main( int argc, char* argv[] ){


	/* �v���O�����J�n�̃R�[�� */
	cout << "----- CSV �f�[�^���� MHA �f�[�^�ւ̕ϊ� -----" << endl;

	/* �R�}���h���C�������ǂݍ��ݎ��s�� */
	if( argc < 3 ){
		cerr << "Caution! : �R�}���h���C�������G���[" << endl;
		cerr << "Usage: " << endl;
		cerr << argv[ 0 ] << " inputFileName.csv outputFileName.mha" << endl;
		return EXIT_FAILURE;
	}

	/* �ϐ��̒�` */
	const char *inputFileName = argv[ 1 ];	// ���̓{�����[���f�[�^��( .csv )
	const char *outputFileName = argv[ 2 ];	// �o�̓{�����[���f�[�^��( .mha, .mhd, �c )
	FILE *inputFile;						// �t�@�C���^�ϐ�
	vector< vector< int > > pixels;		// �{�N�Z���f�[�^�̍��W�l�Ɖ�f�l�̏W�܂�
	vector< int > pixel;					// �{�N�Z���f�[�^�̍��W�l�Ɖ�f�l
	int XSize = 0;
	int YSize = 0;
	int ZSize = 0;
	int largestPixelValue = 0;
	//int temp;
	//int counter = 0;


	/*** �G�N�Z���t�@�C���̓ǂݍ��� ***/

	/* �t�@�C���ǂݍ��ݎ��s�� */
	if( ( inputFile = fopen( inputFileName, "r" ) ) == NULL ){
		cerr << "�t�@�C�� " << inputFileName << " ���J���܂���" << endl;
		return EXIT_FAILURE;
	}

	/* �������̊m�� */
	pixels.resize( 0 );	// 2�����z��̗v�f��
	pixel.resize( 4 );	// 1�����z��̗v�f��

	/* �t�@�C�����e�̓ǂݍ��� */
	cout << ">>> CSV �t�@�C���̓ǂݍ��ݒ�" << "\r";
	while( fscanf( inputFile, "%d,%d,%d,%d", &pixel[ 0 ], &pixel[ 1 ], &pixel[ 2 ], &pixel[ 3 ] ) != EOF ){

#ifdef COMMENT1
		cout << pixel[ 0 ] << " " << pixel[ 1 ] << " " << pixel[ 2 ] << " " << pixel[ 3 ] << endl;
#endif
		/* �{�����[���f�[�^�T�C�Y�̎Z�o */
		if( pixel[ 0 ] > XSize ) XSize = pixel[ 0 ];
		if( pixel[ 1 ] > YSize ) YSize = pixel[ 1 ];
		if( pixel[ 2 ] > ZSize ) ZSize = pixel[ 2 ];
		
		/* �ő��f�l�̎Z�o */
		if( pixel[ 3 ] > largestPixelValue ) largestPixelValue = pixel[ 3 ];
		
		/* 2�����z��ւ�1�����z��̒ǉ� */
		pixels.push_back( pixel );	// �x�N�g���̖����ɑ�1������ǉ�����
	}
	cout << ">>> CSV �t�@�C���̓ǂݍ��� OK" << endl;


#ifdef COMMENT2
	/* �ǂݍ��񂾃{�N�Z���f�[�^�̃R�}���h���C���o�� */
	cout << "�y X, Y, Z, Intensity �z" << endl;
	for( int i = 0; i < pixels.size(); i++ ){
		for( int j = 0; j < pixels[ i ].size(); j++ ){
			cout << pixels[ i ][ j ] << " ";
		}
		cout << endl;
	}
#endif


	/*** �{�����[���f�[�^�̏����o���F3DSlicer �œǂݍ��߂�f�[�^�^�ɕϊ� ***/
	
	/* �f�[�^�^�̒�` */
	typedef int PixelType;	// ���o�͉摜�̃s�N�Z���̃f�[�^�^�Funsigned char
	const unsigned int Dimension = 3;		// ���͉摜�f�[�^�̎������̎w��

	/* �摜�^�̒�` */
	typedef itk::Image< PixelType, Dimension > OutputImageType;

	/* �t�@�C���X�g���[���^�̒�` */
	typedef itk::ImageFileWriter< OutputImageType > WriterType;	// RealImageType �^�̉摜�������o���t�@�C���X�g���[�� 

	/* �������̊m�� */
	OutputImageType::Pointer outputImage = OutputImageType::New();
	WriterType::Pointer writer = WriterType::New();	// �������ݐ�p�t�@�C���X�g���[���^�ϐ�
	
	/* �t�@�C�����̎w�� */
	writer->SetFileName( outputFileName );	// �������ݐ�p�t�@�C���X�g���[���^�ϐ��ɏo�̓t�@�C�������w��

	/* �摜�T�C�Y */
	const OutputImageType::SizeType size = { { XSize, YSize, ZSize } };
	const OutputImageType::IndexType start = { { 0, 0, 0 } };
	int n = 0;

	/* �摜�̈�̎w�� */
	OutputImageType::RegionType region;
	region.SetSize( size );
	region.SetIndex( start );

	outputImage->SetRegions( region );
	outputImage->Allocate();

	OutputImageType::PixelType pixelValue;
	
	/* �{�����[���f�[�^�̌^�ϊ� */
	cout << ">>> �{�����[���f�[�^�̌^�ϊ���" << "\r";
	for( int z = 0; z < ZSize; z++ ){
		for( int y = 0; y < YSize; y++ ){
			for( int x = 0; x < XSize; x++ ){

				OutputImageType::IndexType pixelIndex = { { x, y, z } };
				OutputImageType::PixelType pixelValue = pixels[ n ][ 3 ];
				outputImage->SetPixel( pixelIndex, pixelValue );

				n++;
			}
		}
	}
	cout << ">>> �{�����[���f�[�^�̌^�ϊ� OK" << endl;


	/*** ���ʂ̏o�� ***/
	cout << ">>> �{�����[���f�[�^�o�͒�" << "\r";

	/* �����o����p�t�@�C���X�g���[���^�ϐ��Ƀ{�����[���f�[�^��ݒ� */
	writer->SetInput( outputImage );

	/* �������ʂƂȂ�3�����G�b�W�{�N�Z���f�[�^�̏o�� */
	try{
		writer->Update();
	}

	/* �o�͎��s�� */
	catch( itk::ExceptionObject & err ){
		cout << "ExceptionObject caught !" << endl;
		cout << err << endl;
		return EXIT_FAILURE;
	}
	cout << ">>> �{�����[���f�[�^�o�� OK" << endl;


	return 0;
}

