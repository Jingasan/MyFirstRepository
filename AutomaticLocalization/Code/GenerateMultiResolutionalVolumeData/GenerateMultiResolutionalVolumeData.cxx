/***** ピラミッド探索のための多解像度のマスクボリュームデータの作成 *****/

// 【 コマンドライン引数 】
// 1. 入力ボリュームデータ名.拡張子 
// 2. 出力ボリュームデータ名.拡張子
// 3. ボリュームデータの分解率： 2, 4, 8, … ( オリジナルのボリュームデータの XYZ 各方向の解像度をこの数字で割る )


// 例 1：
// Input/VolumeData/LiverID_0002.mhd Output/MultiResolutionalVolumeData/LiverID_0002/LiverID_0002_0.mha 1

// 例 2：
// Output/MultiResolutionalVolumeData/LiverID_0002/LiverID_0002_0.mha Output/MultiResolutionalVolumeData/LiverID_0002/LiverID_0002_1.mha 2
// Output/MultiResolutionalVolumeData/LiverID_0002/LiverID_0002_1.mha Output/MultiResolutionalVolumeData/LiverID_0002/LiverID_0002_2.mha 2
// Output/MultiResolutionalVolumeData/LiverID_0002/LiverID_0002_2.mha Output/MultiResolutionalVolumeData/LiverID_0002/LiverID_0002_3.mha 2

// 例 3：
// Input/VolumeData/ID_0002.mhd Output/MultiResolutionalVolumeData/ID_0002/ID_0002_0.mha 1

// 例 4：
// Output/MultiResolutionalVolumeData/ID_0002/ID_0002_0.mha Output/MultiResolutionalVolumeData/ID_0002/ID_0002_1.mha 2
// Output/MultiResolutionalVolumeData/ID_0002/ID_0002_1.mha Output/MultiResolutionalVolumeData/ID_0002/ID_0002_2.mha 2
// Output/MultiResolutionalVolumeData/ID_0002/ID_0002_2.mha Output/MultiResolutionalVolumeData/ID_0002/ID_0002_3.mha 2


/*** インクルードファイル ***/

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
#include <time.h>	// 処理時間計測

/* 作成ヘッダファイル */
#include "Configuration.h"		// 設定ヘッダファイル


/*** 名前空間の宣言 ***/
using namespace std;


/*** メイン処理文 ***/
int main( int argc, char* argv[] ){


	/* コマンドライン引数不足時 */
	if( argc < 3 ){
		cerr << "Usage: InputFilename OutputFilename ResolutionLevel[ 1 / ? ]" << endl;
		return EXIT_FAILURE;
	}


	/* プログラム開始のコール */
	cout << "【 プログラム開始 】" << endl;


	/*** 変数の定義、データ型やクラス型の定義 ***/

	/* 変数の初期化 */
	const unsigned int Dimension = 3;	// 次元数：3

	/* データ型の定義 */
#ifdef MODEL_VERSION
	typedef unsigned char PixelType;	// ピクセルのデータ型の定義：unsigned char 型
#else
	typedef short PixelType;			// ピクセルのデータ型の定義：short 型
#endif

	/* 画像型の定義 */
	typedef itk::Image< PixelType, Dimension > ImageType;	// 画像のデータ型の定義
	
	/* ファイルストリーム型の定義 */
	typedef itk::ImageFileReader< ImageType > ReaderType;	// 読み込み専用ファイルストリーム
	typedef itk::ImageFileWriter< ImageType > WriterType;	// 書き込み専用ファイルストリーム
	// 読み込み書き込み共に同じデータ型を指定

	/* メモリの確保 */
	ReaderType::Pointer reader = ReaderType::New();	// 読み込み専用ファイルストリームの型
	WriterType::Pointer writer = WriterType::New();	// 書き込み専用ファイルストリームの型
	
	/* ファイル名の指定 */
	reader->SetFileName( argv[ 1 ] );	// 読み込み用の変数に入力ファイル名を設定
	writer->SetFileName( argv[ 2 ] );	// 書き込み用の変数に出力ファイル名を設定


	/*** 入力ファイルの読み込み ***/
	reader->Update();


	/*** 入力データ情報の取得 ***/

	/* 入力画像の取得 */
	ImageType::Pointer image = reader->GetOutput();
	
	/* 入力ボリュームデータのプロパティのコマンドライン出力 */
	cout << endl << ">> 入力データ情報" << endl;
	cout << "1voxelの X 方向サイズ：" << image->GetSpacing().GetElement( 0 ) << endl;
	cout << "1voxelの Y 方向サイズ：" << image->GetSpacing().GetElement( 1 ) << endl;
	cout << "1voxelの Z 方向サイズ：" << image->GetSpacing().GetElement( 2 ) << endl;
	cout << "ボリュームデータの原点 X 座標：" << image->GetOrigin().GetElement( 0 ) << endl;
	cout << "ボリュームデータの原点 Y 座標：" << image->GetOrigin().GetElement( 1 ) << endl;
	cout << "ボリュームデータの原点 Z 座標：" << image->GetOrigin().GetElement( 2 ) << endl;
	cout << "ボリュームデータの X 方向サイズ：" << image->GetLargestPossibleRegion().GetSize( 0 ) << endl;
	cout << "ボリュームデータの Y 方向サイズ：" << image->GetLargestPossibleRegion().GetSize( 1 ) << endl;
	cout << "ボリュームデータの Z 方向サイズ：" << image->GetLargestPossibleRegion().GetSize( 2 ) << endl << endl;
	
	/* 繰り返し処理用クラス ImageRegionIteratorWithIndex の変数の宣言 */
	itk::ImageRegionIteratorWithIndex< ImageType > imageIterator( image, image->GetLargestPossibleRegion() );	// 座標値と画素値の両方を操作する際に使用


	/* 変数の定義 */
	int x;	// 入力ボリュームデータの X 座標値
	int y;	// 入力ボリュームデータの Y 座標値
	int z;	// 入力ボリュームデータの Z 座標値
	int xSize = image->GetLargestPossibleRegion().GetSize( 0 );	// 入力ボリュームデータの X 方向サイズ
	int ySize = image->GetLargestPossibleRegion().GetSize( 1 );	// 入力ボリュームデータの Y 方向サイズ
	int zSize = image->GetLargestPossibleRegion().GetSize( 2 );	// 入力ボリュームデータの Z 方向サイズ
	int xySize = xSize * ySize;									// 入力ボリュームデータの XY 面積
	int *inputVolumeImage;										// 入力ボリュームデータの画素値
	int *outputVolumeImage;										// 出力ボリュームデータの画素値
	int resolutionLevel = atoi( argv[ 3 ] );					// ボリュームデータの分解率
	int newXSize = xSize / resolutionLevel;						// 出力ボリュームデータの X 方向サイズ
	int newYSize = ySize / resolutionLevel;						// 出力ボリュームデータの Y 方向サイズ
	int newZSize = zSize / resolutionLevel;						// 出力ボリュームデータの Z 方向サイズ
	int newXYSize = newXSize * newYSize;						// 出力ボリュームデータの XY 面積
	int sum;													// 輝度値の和

	/* 解像度レベルのコマンドライン出力 */
	cout << ">> 解像度レベル：1/" << resolutionLevel << endl;

	/* メモリの確保 */
	inputVolumeImage = (int*)malloc( xSize * ySize * zSize * sizeof(int) );				// 入力ボリュームデータ
	outputVolumeImage = (int*)malloc( newXSize * newYSize * newZSize * sizeof(int) );	// 出力ボリュームデータ

	/* ImageIterator の位置を初めに持ってくる */
	imageIterator.GoToBegin();
	
	/* 入力ボリュームデータの輝度値の取得 */
	while( !imageIterator.IsAtEnd() ){

		/* 通し番号の取得 */
		x = imageIterator.GetIndex().GetElement( 0 );	// X 座標
		y = imageIterator.GetIndex().GetElement( 1 );	// Y 座標
		z = imageIterator.GetIndex().GetElement( 2 );	// Z 座標

		/* 分解率 1/1 の場合：なぜかこうしないと出力の Y 座標がミラー状態となる */
		if( resolutionLevel == 1 ){
			inputVolumeImage[ z * xySize + ( ySize - y ) * xSize + x ] = (int)imageIterator.Value();	// 輝度値
		}
		
		/* 分解率 1/2 の場合 */
		else if( resolutionLevel > 1 ){
			inputVolumeImage[ z * xySize + y * xSize + x ] = (int)imageIterator.Value();	// 輝度値
		}

		/* ループの更新 */
		++imageIterator;
	}


	/* 低解像度化 */
	cout << ">> 入力ボリュームデータの低解像度化開始" << "\r";
	for( z = 0; z < newZSize; z++ ){
		for( y = 0; y < newYSize; y++ ){
			for( x = 0; x < newXSize; x++ ){
				
				/* 注目領域内の画素値の和の初期化 */
				sum = 0;
				
				for( int k = 0; k < resolutionLevel; k++ ){
					for( int j = 0; j < resolutionLevel; j++ ){
						for( int i = 0; i < resolutionLevel; i++ ){
							
							/* 分解率 × 分解率 × 分解率の注目領域内のボクセルの輝度値の和の算出 */
							sum += inputVolumeImage[ ( z * resolutionLevel + k ) * xySize + ( y * resolutionLevel + j ) * xSize + ( x * resolutionLevel + i ) ];
						}
					}
				}
				
				/* 平均値の算出 */
				outputVolumeImage[ z * newXYSize + y * newXSize + x ] = sum / ( resolutionLevel * resolutionLevel * resolutionLevel );
			}
		}
	}
	cout << ">> 入力ボリュームデータの低解像度化完了" << endl;


	/*** 出力ボリュームデータの作成 ***/

	/* ImportFilterType クラスのオブジェクト変数の定義とメモリの確保 */
	typedef itk::ImportImageFilter< PixelType, Dimension > ImportFilterType;
	ImportFilterType::Pointer importFilter = ImportFilterType::New();

	/* 画像領域の設定 */
	ImportFilterType::SizeType size;		// サイズ格納用変数
	size[ 0 ] = newXSize;					// X 方向領域サイズの指定
	size[ 1 ] = newYSize;					// Y 方向領域サイズの指定
	size[ 2 ] = newZSize;					// Z 方向領域サイズの指定
	ImportFilterType::IndexType start;		// 始点インデックス格納用変数
	start.Fill( 0 );						// 始点インデックスに 0 を指定
	ImportFilterType::RegionType region;	// 領域情報の格納用変数
	region.SetIndex( start );				// 領域始点の設定
	region.SetSize( size );					// 領域サイズの設定
	importFilter->SetRegion( region );		// 領域の設定

	/* 原点座標の設定 */
	importFilter->SetOrigin( image->GetOrigin() );

	/* ボクセル間隔の設定 */
	importFilter->SetSpacing( image->GetSpacing() );

	/* ボクセル数の設定とメモリの確保 */
	const unsigned int numberOfPixels = size[ 0 ] * size[ 1 ] * size[ 2 ];	// ボクセル数
	PixelType *localBuffer = new PixelType[ numberOfPixels ];				// ImportImageFilter に渡す画素値データを格納するメモリ領域を確保
	
	/* ボリューム格納用変数 */
	PixelType *it = localBuffer;

	for( z = 0; z < newZSize; z++ ){
		for( y = 0; y < newYSize; y++ ){
			for( x = 0; x < newXSize; x++ ){
				
				//if( outputVolumeImage[ z * newXYSize + y * newXSize + x ] == 1 ) cout << outputVolumeImage[ z * newXYSize + y * newXSize + x ] << endl;
		
#ifdef MODEL_VERSION

				/* 画素値（ 0 or 255 の二値 ）の指定 */
				if( outputVolumeImage[ z * newXYSize + y * newXSize + x ] > 0 ){
					*it++ = VOXEL_VALUE;
				}else{
					*it++ = 0;
				}
#else
				/* 画素値をそのまま格納 */
				*it++ = outputVolumeImage[ z * newXYSize + y * newXSize + x ];
#endif
			}
		}
	}

	/* フラグの定義 */
	const bool importImageFilterWillOwnTheBuffer = true;
	
	/* ImportImageFilter に作成したボリュームデータを設定 */
	importFilter->SetImportPointer( localBuffer, numberOfPixels, importImageFilterWillOwnTheBuffer );
	// 第1引数：画素値データを格納したポインタ配列
	// 第2引数：ボクセル数
	// 第3引数：ImportImageFilter 使用後に、画素値が格納されたメモリブロックを破棄するかどうかのフラグ( true：破棄する, false：破棄しない )
	//			ture 値を設定することで、画素値が格納されたメモリを最後に delete で破棄する必要がなくなる



	/*** 低解像度化ボリュームデータの出力 ***/
	cout << ">> 低解像度化ボリュームデータの出力中" << "\r";
	
	/* 処理結果画像を設定 */
	writer->SetInput( importFilter->GetOutput() );

	/* 処理結果画像の出力 */
	try{
		writer->Update();
    }

	/* 書き込み失敗時 */
	catch ( itk::ExceptionObject &err ){
		cerr << "ExceptionObject caught !" << endl;
		cerr << err << endl;
		return -1;
    }
	cout << ">> 低解像度化ボリュームデータの出力完了" << endl;


	/*** 出力データ情報の取得 ***/

	/* 出力画像の取得 */
	ImageType::Pointer outputImage = importFilter->GetOutput();

	/* 出力ボリュームデータのプロパティのコマンドライン出力 */
	cout << endl << ">> 出力データ情報" << endl;
	cout << "1voxelの X 方向サイズ：" << outputImage->GetSpacing().GetElement( 0 ) << endl;
	cout << "1voxelの Y 方向サイズ：" << outputImage->GetSpacing().GetElement( 1 ) << endl;
	cout << "1voxelの Z 方向サイズ：" << outputImage->GetSpacing().GetElement( 2 ) << endl;
	cout << "ボリュームデータの原点 X 座標：" << outputImage->GetOrigin().GetElement( 0 ) << endl;
	cout << "ボリュームデータの原点 Y 座標：" << outputImage->GetOrigin().GetElement( 1 ) << endl;
	cout << "ボリュームデータの原点 Z 座標：" << outputImage->GetOrigin().GetElement( 2 ) << endl;
	cout << "ボリュームデータの X 方向サイズ：" << outputImage->GetLargestPossibleRegion().GetSize( 0 ) << endl;
	cout << "ボリュームデータの Y 方向サイズ：" << outputImage->GetLargestPossibleRegion().GetSize( 1 ) << endl;
	cout << "ボリュームデータの Z 方向サイズ：" << outputImage->GetLargestPossibleRegion().GetSize( 2 ) << endl << endl;
	
	/* プログラム終了のコール */
	cout << "【 プログラム終了 】" << endl;


	return EXIT_SUCCESS;
}


