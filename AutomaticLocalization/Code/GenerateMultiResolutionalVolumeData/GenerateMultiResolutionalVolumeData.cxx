/***** �s���~�b�h�T���̂��߂̑��𑜓x�̃}�X�N�{�����[���f�[�^�̍쐬 *****/

// �y �R�}���h���C������ �z
// 1. ���̓{�����[���f�[�^��.�g���q 
// 2. �o�̓{�����[���f�[�^��.�g���q
// 3. �{�����[���f�[�^�̕��𗦁F 2, 4, 8, �c ( �I���W�i���̃{�����[���f�[�^�� XYZ �e�����̉𑜓x�����̐����Ŋ��� )


// �� 1�F
// Input/VolumeData/LiverID_0002.mhd Output/MultiResolutionalVolumeData/LiverID_0002/LiverID_0002_0.mha 1

// �� 2�F
// Output/MultiResolutionalVolumeData/LiverID_0002/LiverID_0002_0.mha Output/MultiResolutionalVolumeData/LiverID_0002/LiverID_0002_1.mha 2
// Output/MultiResolutionalVolumeData/LiverID_0002/LiverID_0002_1.mha Output/MultiResolutionalVolumeData/LiverID_0002/LiverID_0002_2.mha 2
// Output/MultiResolutionalVolumeData/LiverID_0002/LiverID_0002_2.mha Output/MultiResolutionalVolumeData/LiverID_0002/LiverID_0002_3.mha 2

// �� 3�F
// Input/VolumeData/ID_0002.mhd Output/MultiResolutionalVolumeData/ID_0002/ID_0002_0.mha 1

// �� 4�F
// Output/MultiResolutionalVolumeData/ID_0002/ID_0002_0.mha Output/MultiResolutionalVolumeData/ID_0002/ID_0002_1.mha 2
// Output/MultiResolutionalVolumeData/ID_0002/ID_0002_1.mha Output/MultiResolutionalVolumeData/ID_0002/ID_0002_2.mha 2
// Output/MultiResolutionalVolumeData/ID_0002/ID_0002_2.mha Output/MultiResolutionalVolumeData/ID_0002/ID_0002_3.mha 2


/*** �C���N���[�h�t�@�C�� ***/

/* ITK 4.5.2 */
#include "itkImage.h"
#include "itkImageFileReader.h"
#include "itkImageFileWriter.h"
#include "itkImportImageFilter.h"
#include "itkImageRegionIteratorWithIndex.h"

/* C++ */
#include <iostream>
#include <fstream>

/* C */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <float.h>

/* C */
#include <time.h>	// �������Ԍv��

/* �쐬�w�b�_�t�@�C�� */
#include "Configuration.h"		// �ݒ�w�b�_�t�@�C��


/*** ���O��Ԃ̐錾 ***/
using namespace std;


/*** ���C�������� ***/
int main( int argc, char* argv[] ){


	/* �R�}���h���C�������s���� */
	if( argc < 3 ){
		cerr << "Usage: InputFilename OutputFilename ResolutionLevel[ 1 / ? ]" << endl;
		return EXIT_FAILURE;
	}


	/* �v���O�����J�n�̃R�[�� */
	cout << "�y �v���O�����J�n �z" << endl;


	/*** �ϐ��̒�`�A�f�[�^�^��N���X�^�̒�` ***/

	/* �ϐ��̏����� */
	const unsigned int Dimension = 3;	// �������F3

	/* �f�[�^�^�̒�` */
#ifdef MODEL_VERSION
	typedef unsigned char PixelType;	// �s�N�Z���̃f�[�^�^�̒�`�Funsigned char �^
#else
	typedef short PixelType;			// �s�N�Z���̃f�[�^�^�̒�`�Fshort �^
#endif

	/* �摜�^�̒�` */
	typedef itk::Image< PixelType, Dimension > ImageType;	// �摜�̃f�[�^�^�̒�`
	
	/* �t�@�C���X�g���[���^�̒�` */
	typedef itk::ImageFileReader< ImageType > ReaderType;	// �ǂݍ��ݐ�p�t�@�C���X�g���[��
	typedef itk::ImageFileWriter< ImageType > WriterType;	// �������ݐ�p�t�@�C���X�g���[��
	// �ǂݍ��ݏ������݋��ɓ����f�[�^�^���w��

	/* �������̊m�� */
	ReaderType::Pointer reader = ReaderType::New();	// �ǂݍ��ݐ�p�t�@�C���X�g���[���̌^
	WriterType::Pointer writer = WriterType::New();	// �������ݐ�p�t�@�C���X�g���[���̌^
	
	/* �t�@�C�����̎w�� */
	reader->SetFileName( argv[ 1 ] );	// �ǂݍ��ݗp�̕ϐ��ɓ��̓t�@�C������ݒ�
	writer->SetFileName( argv[ 2 ] );	// �������ݗp�̕ϐ��ɏo�̓t�@�C������ݒ�


	/*** ���̓t�@�C���̓ǂݍ��� ***/
	reader->Update();


	/*** ���̓f�[�^���̎擾 ***/

	/* ���͉摜�̎擾 */
	ImageType::Pointer image = reader->GetOutput();
	
	/* ���̓{�����[���f�[�^�̃v���p�e�B�̃R�}���h���C���o�� */
	cout << endl << ">> ���̓f�[�^���" << endl;
	cout << "1voxel�� X �����T�C�Y�F" << image->GetSpacing().GetElement( 0 ) << endl;
	cout << "1voxel�� Y �����T�C�Y�F" << image->GetSpacing().GetElement( 1 ) << endl;
	cout << "1voxel�� Z �����T�C�Y�F" << image->GetSpacing().GetElement( 2 ) << endl;
	cout << "�{�����[���f�[�^�̌��_ X ���W�F" << image->GetOrigin().GetElement( 0 ) << endl;
	cout << "�{�����[���f�[�^�̌��_ Y ���W�F" << image->GetOrigin().GetElement( 1 ) << endl;
	cout << "�{�����[���f�[�^�̌��_ Z ���W�F" << image->GetOrigin().GetElement( 2 ) << endl;
	cout << "�{�����[���f�[�^�� X �����T�C�Y�F" << image->GetLargestPossibleRegion().GetSize( 0 ) << endl;
	cout << "�{�����[���f�[�^�� Y �����T�C�Y�F" << image->GetLargestPossibleRegion().GetSize( 1 ) << endl;
	cout << "�{�����[���f�[�^�� Z �����T�C�Y�F" << image->GetLargestPossibleRegion().GetSize( 2 ) << endl << endl;
	
	/* �J��Ԃ������p�N���X ImageRegionIteratorWithIndex �̕ϐ��̐錾 */
	itk::ImageRegionIteratorWithIndex< ImageType > imageIterator( image, image->GetLargestPossibleRegion() );	// ���W�l�Ɖ�f�l�̗����𑀍삷��ۂɎg�p


	/* �ϐ��̒�` */
	int x;	// ���̓{�����[���f�[�^�� X ���W�l
	int y;	// ���̓{�����[���f�[�^�� Y ���W�l
	int z;	// ���̓{�����[���f�[�^�� Z ���W�l
	int xSize = image->GetLargestPossibleRegion().GetSize( 0 );	// ���̓{�����[���f�[�^�� X �����T�C�Y
	int ySize = image->GetLargestPossibleRegion().GetSize( 1 );	// ���̓{�����[���f�[�^�� Y �����T�C�Y
	int zSize = image->GetLargestPossibleRegion().GetSize( 2 );	// ���̓{�����[���f�[�^�� Z �����T�C�Y
	int xySize = xSize * ySize;									// ���̓{�����[���f�[�^�� XY �ʐ�
	int *inputVolumeImage;										// ���̓{�����[���f�[�^�̉�f�l
	int *outputVolumeImage;										// �o�̓{�����[���f�[�^�̉�f�l
	int resolutionLevel = atoi( argv[ 3 ] );					// �{�����[���f�[�^�̕���
	int newXSize = xSize / resolutionLevel;						// �o�̓{�����[���f�[�^�� X �����T�C�Y
	int newYSize = ySize / resolutionLevel;						// �o�̓{�����[���f�[�^�� Y �����T�C�Y
	int newZSize = zSize / resolutionLevel;						// �o�̓{�����[���f�[�^�� Z �����T�C�Y
	int newXYSize = newXSize * newYSize;						// �o�̓{�����[���f�[�^�� XY �ʐ�
	int sum;													// �P�x�l�̘a

	/* �𑜓x���x���̃R�}���h���C���o�� */
	cout << ">> �𑜓x���x���F1/" << resolutionLevel << endl;

	/* �������̊m�� */
	inputVolumeImage = (int*)malloc( xSize * ySize * zSize * sizeof(int) );				// ���̓{�����[���f�[�^
	outputVolumeImage = (int*)malloc( newXSize * newYSize * newZSize * sizeof(int) );	// �o�̓{�����[���f�[�^

	/* ImageIterator �̈ʒu�����߂Ɏ����Ă��� */
	imageIterator.GoToBegin();
	
	/* ���̓{�����[���f�[�^�̋P�x�l�̎擾 */
	while( !imageIterator.IsAtEnd() ){

		/* �ʂ��ԍ��̎擾 */
		x = imageIterator.GetIndex().GetElement( 0 );	// X ���W
		y = imageIterator.GetIndex().GetElement( 1 );	// Y ���W
		z = imageIterator.GetIndex().GetElement( 2 );	// Z ���W

		/* ���� 1/1 �̏ꍇ�F�Ȃ����������Ȃ��Əo�͂� Y ���W���~���[��ԂƂȂ� */
		if( resolutionLevel == 1 ){
			inputVolumeImage[ z * xySize + ( ySize - y ) * xSize + x ] = (int)imageIterator.Value();	// �P�x�l
		}
		
		/* ���� 1/2 �̏ꍇ */
		else if( resolutionLevel > 1 ){
			inputVolumeImage[ z * xySize + y * xSize + x ] = (int)imageIterator.Value();	// �P�x�l
		}

		/* ���[�v�̍X�V */
		++imageIterator;
	}


	/* ��𑜓x�� */
	cout << ">> ���̓{�����[���f�[�^�̒�𑜓x���J�n" << "\r";
	for( z = 0; z < newZSize; z++ ){
		for( y = 0; y < newYSize; y++ ){
			for( x = 0; x < newXSize; x++ ){
				
				/* ���ڗ̈���̉�f�l�̘a�̏����� */
				sum = 0;
				
				for( int k = 0; k < resolutionLevel; k++ ){
					for( int j = 0; j < resolutionLevel; j++ ){
						for( int i = 0; i < resolutionLevel; i++ ){
							
							/* ���� �~ ���� �~ ���𗦂̒��ڗ̈���̃{�N�Z���̋P�x�l�̘a�̎Z�o */
							sum += inputVolumeImage[ ( z * resolutionLevel + k ) * xySize + ( y * resolutionLevel + j ) * xSize + ( x * resolutionLevel + i ) ];
						}
					}
				}
				
				/* ���ϒl�̎Z�o */
				outputVolumeImage[ z * newXYSize + y * newXSize + x ] = sum / ( resolutionLevel * resolutionLevel * resolutionLevel );
			}
		}
	}
	cout << ">> ���̓{�����[���f�[�^�̒�𑜓x������" << endl;


	/*** �o�̓{�����[���f�[�^�̍쐬 ***/

	/* ImportFilterType �N���X�̃I�u�W�F�N�g�ϐ��̒�`�ƃ������̊m�� */
	typedef itk::ImportImageFilter< PixelType, Dimension > ImportFilterType;
	ImportFilterType::Pointer importFilter = ImportFilterType::New();

	/* �摜�̈�̐ݒ� */
	ImportFilterType::SizeType size;		// �T�C�Y�i�[�p�ϐ�
	size[ 0 ] = newXSize;					// X �����̈�T�C�Y�̎w��
	size[ 1 ] = newYSize;					// Y �����̈�T�C�Y�̎w��
	size[ 2 ] = newZSize;					// Z �����̈�T�C�Y�̎w��
	ImportFilterType::IndexType start;		// �n�_�C���f�b�N�X�i�[�p�ϐ�
	start.Fill( 0 );						// �n�_�C���f�b�N�X�� 0 ���w��
	ImportFilterType::RegionType region;	// �̈���̊i�[�p�ϐ�
	region.SetIndex( start );				// �̈�n�_�̐ݒ�
	region.SetSize( size );					// �̈�T�C�Y�̐ݒ�
	importFilter->SetRegion( region );		// �̈�̐ݒ�

	/* ���_���W�̐ݒ� */
	importFilter->SetOrigin( image->GetOrigin() );

	/* �{�N�Z���Ԋu�̐ݒ� */
	importFilter->SetSpacing( image->GetSpacing() );

	/* �{�N�Z�����̐ݒ�ƃ������̊m�� */
	const unsigned int numberOfPixels = size[ 0 ] * size[ 1 ] * size[ 2 ];	// �{�N�Z����
	PixelType *localBuffer = new PixelType[ numberOfPixels ];				// ImportImageFilter �ɓn����f�l�f�[�^���i�[���郁�����̈���m��
	
	/* �{�����[���i�[�p�ϐ� */
	PixelType *it = localBuffer;

	for( z = 0; z < newZSize; z++ ){
		for( y = 0; y < newYSize; y++ ){
			for( x = 0; x < newXSize; x++ ){
				
				//if( outputVolumeImage[ z * newXYSize + y * newXSize + x ] == 1 ) cout << outputVolumeImage[ z * newXYSize + y * newXSize + x ] << endl;
		
#ifdef MODEL_VERSION

				/* ��f�l�i 0 or 255 �̓�l �j�̎w�� */
				if( outputVolumeImage[ z * newXYSize + y * newXSize + x ] > 0 ){
					*it++ = VOXEL_VALUE;
				}else{
					*it++ = 0;
				}
#else
				/* ��f�l�����̂܂܊i�[ */
				*it++ = outputVolumeImage[ z * newXYSize + y * newXSize + x ];
#endif
			}
		}
	}

	/* �t���O�̒�` */
	const bool importImageFilterWillOwnTheBuffer = true;
	
	/* ImportImageFilter �ɍ쐬�����{�����[���f�[�^��ݒ� */
	importFilter->SetImportPointer( localBuffer, numberOfPixels, importImageFilterWillOwnTheBuffer );
	// ��1�����F��f�l�f�[�^���i�[�����|�C���^�z��
	// ��2�����F�{�N�Z����
	// ��3�����FImportImageFilter �g�p��ɁA��f�l���i�[���ꂽ�������u���b�N��j�����邩�ǂ����̃t���O( true�F�j������, false�F�j�����Ȃ� )
	//			ture �l��ݒ肷�邱�ƂŁA��f�l���i�[���ꂽ���������Ō�� delete �Ŕj������K�v���Ȃ��Ȃ�



	/*** ��𑜓x���{�����[���f�[�^�̏o�� ***/
	cout << ">> ��𑜓x���{�����[���f�[�^�̏o�͒�" << "\r";
	
	/* �������ʉ摜��ݒ� */
	writer->SetInput( importFilter->GetOutput() );

	/* �������ʉ摜�̏o�� */
	try{
		writer->Update();
    }

	/* �������ݎ��s�� */
	catch ( itk::ExceptionObject &err ){
		cerr << "ExceptionObject caught !" << endl;
		cerr << err << endl;
		return -1;
    }
	cout << ">> ��𑜓x���{�����[���f�[�^�̏o�͊���" << endl;


	/*** �o�̓f�[�^���̎擾 ***/

	/* �o�͉摜�̎擾 */
	ImageType::Pointer outputImage = importFilter->GetOutput();

	/* �o�̓{�����[���f�[�^�̃v���p�e�B�̃R�}���h���C���o�� */
	cout << endl << ">> �o�̓f�[�^���" << endl;
	cout << "1voxel�� X �����T�C�Y�F" << outputImage->GetSpacing().GetElement( 0 ) << endl;
	cout << "1voxel�� Y �����T�C�Y�F" << outputImage->GetSpacing().GetElement( 1 ) << endl;
	cout << "1voxel�� Z �����T�C�Y�F" << outputImage->GetSpacing().GetElement( 2 ) << endl;
	cout << "�{�����[���f�[�^�̌��_ X ���W�F" << outputImage->GetOrigin().GetElement( 0 ) << endl;
	cout << "�{�����[���f�[�^�̌��_ Y ���W�F" << outputImage->GetOrigin().GetElement( 1 ) << endl;
	cout << "�{�����[���f�[�^�̌��_ Z ���W�F" << outputImage->GetOrigin().GetElement( 2 ) << endl;
	cout << "�{�����[���f�[�^�� X �����T�C�Y�F" << outputImage->GetLargestPossibleRegion().GetSize( 0 ) << endl;
	cout << "�{�����[���f�[�^�� Y �����T�C�Y�F" << outputImage->GetLargestPossibleRegion().GetSize( 1 ) << endl;
	cout << "�{�����[���f�[�^�� Z �����T�C�Y�F" << outputImage->GetLargestPossibleRegion().GetSize( 2 ) << endl << endl;
	
	/* �v���O�����I���̃R�[�� */
	cout << "�y �v���O�����I�� �z" << endl;


	return EXIT_SUCCESS;
}


